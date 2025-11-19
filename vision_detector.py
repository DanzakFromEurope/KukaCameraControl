import cv2
import numpy as np
import os

# --- CONFIGURATION ---
# Update these values based on what you find in 'vision_tuner.py'
CONFIG = {
    'threshold_value': 100,  # 0-255: Controls sensitivity to dark/light
    'min_area': 1000,  # Minimum size of object (pixels)
    'max_area': 10000,  # Maximum size of object (pixels)
    'debug_mode': True  # Set to True to see the black/white mask and console prints
}


def find_and_draw_cubes(frame):
    """
    Takes a single image frame, finds cubes, draws them, and returns the modified frame.
    """
    # 1. Preprocessing
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Thresholding
    # We use THRESH_BINARY_INV because usually we detect dark objects on light backgrounds.
    # This turns dark objects WHITE and background BLACK (which is what findContours needs).
    _, thresh = cv2.threshold(blurred, CONFIG['threshold_value'], 255, cv2.THRESH_BINARY_INV)

    # Clean up noise
    kernel = np.ones((3, 3), np.uint8)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=2)

    # Show the mask if debugging (Helps you see what the computer sees)
    if CONFIG['debug_mode']:
        cv2.imshow('Debug Mask (White = Detected)', thresh)

    # 2. Find Contours
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if CONFIG['debug_mode']:
        print(f"--- Frame Analysis ---")
        print(f"Found {len(contours)} raw contours.")

    detected_count = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)

        # Debug print for every contour
        if CONFIG['debug_mode'] and area > 100:  # Only print significant specks
            print(f"Checking contour with area: {area:.1f}")

        if CONFIG['min_area'] < area < CONFIG['max_area']:
            detected_count += 1

            # 3. Geometry & Orientation
            rect = cv2.minAreaRect(cnt)
            (center_x, center_y), (width, height), angle = rect

            # Normalize Angle logic
            if width < height:
                angle = angle + 90
                width, height = height, width

            # --- VISUALIZATION ---
            box = cv2.boxPoints(rect)
            # FIX: np.int0 is deprecated in NumPy 1.24+. Use np.int32 instead.
            box = np.int32(box)

            # Draw Box (Green)
            cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)

            # Draw Orientation Line (Blue)
            import math
            angle_rad = math.radians(angle)
            line_len = 40
            end_x = int(center_x + line_len * math.cos(angle_rad))
            end_y = int(center_y + line_len * math.sin(angle_rad))
            cv2.line(frame, (int(center_x), int(center_y)), (end_x, end_y), (255, 0, 0), 2)

            # Text Info
            info_text = f"({int(center_x)},{int(center_y)}) {int(angle)}deg"
            cv2.putText(frame, info_text, (int(center_x) - 40, int(center_y) - 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

    if CONFIG['debug_mode'] and detected_count == 0:
        print("WARNING: No cubes detected. Try adjusting 'threshold_value' or 'min_area'.")

    return frame


def run_vision_system(source):
    """
    Detects if source is an image file, video file, or camera index
    and runs the appropriate loop.
    """
    # CASE 1: Source is an Integer -> Webcam
    if isinstance(source, int):
        cap = cv2.VideoCapture(source)
        print(f"Starting Webcam {source}...")
        while True:
            ret, frame = cap.read()
            if not ret: break

            processed_frame = find_and_draw_cubes(frame)
            cv2.imshow('Robot Vision System', processed_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cap.release()

    # CASE 2: Source is a String -> File
    elif isinstance(source, str):
        # Check if file exists
        if not os.path.exists(source):
            print(f"Error: File '{source}' not found.")
            return

        # Check extension to see if it's an Image
        image_extensions = ['.png', '.jpg', '.jpeg', '.bmp', '.tiff']
        if any(source.lower().endswith(ext) for ext in image_extensions):
            print(f"Processing Single Image: {source}")
            frame = cv2.imread(source)

            if frame is None:
                print("Error: Could not read image.")
                return

            processed_frame = find_and_draw_cubes(frame)

            # Show image and wait indefinitely until a key is pressed
            cv2.imshow('Robot Vision System (Static)', processed_frame)
            print("Press any key to close window...")
            cv2.waitKey(0)

            # Otherwise assume it is a Video
        else:
            print(f"Processing Video File: {source}")
            cap = cv2.VideoCapture(source)
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("Video ended.")
                    break

                processed_frame = find_and_draw_cubes(frame)
                cv2.imshow('Robot Vision System', processed_frame)

                # Wait 25ms between frames (approx 40fps)
                if cv2.waitKey(25) & 0xFF == ord('q'):
                    break
            cap.release()

    cv2.destroyAllWindows()