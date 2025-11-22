from ultralytics import YOLO
import cv2
import math
import numpy as np
import os
import torch

# --- CONFIGURATION ---
MODEL_PATH = 'best.pt'
SOURCE = 'test_images/camera_image_14.png'  # 0 for Webcam, 'video.mp4', or 'image.png'

# Set this to True to save the results to disk
SAVE_OUTPUT = True

# Set this to True to simulate Industrial Camera (Gray) on a Webcam
SIMULATE_BAUMER = True


def run_yolo_inference():
    global SAVE_OUTPUT

    # 0. Select Device
    if torch.backends.mps.is_available():
        device = 'mps'
        print("Using Apple MPS acceleration!")
    elif torch.cuda.is_available():
        device = 0
        print("Using NVIDIA GPU acceleration!")
    else:
        device = 'cpu'
        print("Using CPU.")

    # 1. Load the Model
    try:
        model = YOLO(MODEL_PATH)
    except:
        print(f"Error: Could not load '{MODEL_PATH}'.")
        print("You need to TRAIN the model first! (See 'yolo_training_guide.md')")
        return

    # 2. Open Source Manually
    is_static_image = False
    cap = None
    frame_static = None

    if isinstance(SOURCE, str) and any(SOURCE.lower().endswith(ext) for ext in ['.png', '.jpg', '.jpeg', '.bmp']):
        is_static_image = True
        frame_static = cv2.imread(SOURCE)
        if frame_static is None:
            print(f"Error: Could not read image {SOURCE}")
            return
    else:
        cap = cv2.VideoCapture(SOURCE)
        if not cap.isOpened():
            print(f"Error: Could not open video source {SOURCE}")
            return

    print(f"Starting YOLOv8 OBB Inference on: {SOURCE}")
    print("Press 'q' to exit.")

    # Output Writer Setup
    video_writer = None
    output_filename = "inference_result.png" if is_static_image else "inference_output.mp4"

    while True:
        # 3. Get Frame
        if is_static_image:
            frame = frame_static.copy()
        else:
            ret, frame = cap.read()
            if not ret:
                if isinstance(SOURCE, str):
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
                break

        # 4. BAUMER SIMULATION
        if SIMULATE_BAUMER:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        # 5. Run Inference
        results = model.predict(frame, verbose=False, device=device)

        # --- NEW: COLLECT & SORT CUBES ---
        detected_cubes = []

        for result in results:
            obbs = result.obb
            if obbs is not None:
                for obb in obbs:
                    box = obb.xywhr[0].cpu().numpy()
                    cx, cy, w, h, rot_rad = box
                    rot_deg = math.degrees(rot_rad)

                    detected_cubes.append({
                        'cx': cx, 'cy': cy, 'w': w, 'h': h,
                        'rot_rad': rot_rad, 'rot_deg': rot_deg
                    })

        # SORT CUBES LEFT-TO-RIGHT (Based on X coordinate)
        detected_cubes.sort(key=lambda c: c['cx'])

        # --- DRAWING ---

        # Calibration Status Message
        num_cubes = len(detected_cubes)
        if num_cubes < 4:
            cv2.putText(frame, f"WARNING: Found {num_cubes}/4 Cubes", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        else:
            cv2.putText(frame, f"Calibration Ready: {num_cubes} Cubes", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # Draw Sorted Cubes
        for i, cube in enumerate(detected_cubes):
            cx, cy = cube['cx'], cube['cy']
            w, h = cube['w'], cube['h']
            rot_rad = cube['rot_rad']
            rot_deg = cube['rot_deg']

            # 1. Box
            rect = ((cx, cy), (w, h), rot_deg)
            box_points = cv2.boxPoints(rect)
            box_points = np.int32(box_points)
            cv2.drawContours(frame, [box_points], 0, (0, 255, 0), 2)

            # 2. ID Number (Purple, Large)
            # Drawn to the top-left of the center
            cv2.putText(frame, f"#{i + 1}", (int(cx) - 40, int(cy) - 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 255), 3)

            # 3. Center Point
            center = (int(cx), int(cy))
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

            # 4. Angle Indicator Line
            line_len = 40
            end_x = int(cx + line_len * math.cos(rot_rad))
            end_y = int(cy + line_len * math.sin(rot_rad))
            cv2.line(frame, center, (end_x, end_y), (255, 0, 0), 2)

            # 5. Angle Text
            info_text = f"{int(rot_deg)} deg"
            cv2.putText(frame, info_text, (int(cx), int(cy) + 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # --- SAVE OUTPUT LOGIC ---
        if SAVE_OUTPUT:
            if is_static_image:
                cv2.imwrite(output_filename, frame)
                print(f"Image saved to {output_filename}")
                SAVE_OUTPUT = False
            else:
                if video_writer is None:
                    height, width, _ = frame.shape
                    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                    video_writer = cv2.VideoWriter(output_filename, fourcc, 30.0, (width, height))
                video_writer.write(frame)

        # Display
        if SIMULATE_BAUMER:
            cv2.putText(frame, "SIMULATION: MONO8 MODE", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        cv2.imshow("YOLOv8 OBB Robot Feed", frame)

        delay = 0 if is_static_image else 1
        key = cv2.waitKey(delay)
        if key & 0xFF == ord('q'):
            break

    if video_writer:
        video_writer.release()
        print(f"Video saved to {output_filename}")

    if cap:
        cap.release()

    cv2.destroyAllWindows()


if __name__ == "__main__":
    run_yolo_inference()