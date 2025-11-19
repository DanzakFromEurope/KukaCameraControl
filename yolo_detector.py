from ultralytics import YOLO
import cv2
import math
import numpy as np
import os
import torch

# --- CONFIGURATION ---
MODEL_PATH = 'best.pt'
SOURCE = 'test_images/camera_video.mp4'  # Examples: 0 for webcam, 'video.mp4', or 'image.png'

# Set this to True to save the results to disk
SAVE_OUTPUT = True


def run_yolo_inference():
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

    # 2. Determine input type
    is_static_image = False
    if isinstance(SOURCE, str) and any(SOURCE.lower().endswith(ext) for ext in ['.png', '.jpg', '.jpeg', '.bmp']):
        is_static_image = True

    print(f"Starting YOLOv8 OBB Inference on: {SOURCE}")
    print("Press 'q' to exit.")

    # 3. Run Inference
    results_generator = model.predict(source=SOURCE, stream=True, verbose=False, device=device)

    # Output Writer Setup
    video_writer = None
    output_filename = "inference_result.png" if is_static_image else "inference_output.mp4"

    for result in results_generator:
        frame = result.orig_img

        # Get Oriented Bounding Boxes
        obbs = result.obb

        if obbs is not None:
            for obb in obbs:
                # Extract data: center_x, center_y, width, height, rotation (radians)
                box = obb.xywhr[0].cpu().numpy()
                cx, cy, w, h, rot_rad = box

                rot_deg = math.degrees(rot_rad)

                # --- DRAWING ---
                # 1. Box
                rect = ((cx, cy), (w, h), rot_deg)
                box_points = cv2.boxPoints(rect)
                box_points = np.int32(box_points)
                cv2.drawContours(frame, [box_points], 0, (0, 255, 0), 2)

                # 2. Center Point
                center = (int(cx), int(cy))
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

                # 3. Angle Indicator Line
                line_len = 40
                end_x = int(cx + line_len * math.cos(rot_rad))
                end_y = int(cy + line_len * math.sin(rot_rad))
                cv2.line(frame, center, (end_x, end_y), (255, 0, 0), 2)

                # 4. Text
                info_text = f"{int(rot_deg)} deg"
                cv2.putText(frame, info_text, (int(cx), int(cy) - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # --- SAVE OUTPUT LOGIC ---
        if SAVE_OUTPUT:
            if is_static_image:
                cv2.imwrite(output_filename, frame)
                print(f"Image saved to {output_filename}")
            else:
                # Lazy initialization of video writer (needs frame size)
                if video_writer is None:
                    height, width, _ = frame.shape
                    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                    # 30 FPS is standard, adjust if needed
                    video_writer = cv2.VideoWriter(output_filename, fourcc, 30.0, (width, height))

                video_writer.write(frame)

        cv2.imshow("YOLOv8 OBB Robot Feed", frame)

        # Wait Logic
        delay = 0 if is_static_image else 1
        key = cv2.waitKey(delay)
        if key & 0xFF == ord('q'):
            break

    # Cleanup
    if video_writer:
        video_writer.release()
        print(f"Video saved to {output_filename}")

    cv2.destroyAllWindows()


if __name__ == "__main__":
    run_yolo_inference()