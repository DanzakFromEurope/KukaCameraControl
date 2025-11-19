import cv2
import numpy as np
import os

# --- INSTRUCTIONS ---
# 1. Adjust 'Gamma' first to make the top faces distinct from the sides.
# 2. Adjust 'Squareness' to ignore "long" rectangular shapes.
# 3. Adjust 'Dimple Thresh' and Area Min/Max to isolate the circles.

SOURCE = 'test_images/camera_image_14.png'


def nothing(x):
    pass


def apply_gamma(image, gamma=1.0):
    # Build a lookup table mapping the pixel values [0, 255] to their adjusted gamma values
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255 for i in np.arange(0, 256)]).astype("uint8")
    return cv2.LUT(image, table)


def run_tuner():
    cv2.namedWindow('Tuner', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Tuner', 1400, 900)

    # --- SLIDERS ---
    cv2.createTrackbar('Gamma', 'Tuner', 100, 300, nothing)
    cv2.createTrackbar('Squareness', 'Tuner', 30, 100, nothing)

    cv2.createTrackbar('Paper Thresh', 'Tuner', 150, 255, nothing)

    cv2.createTrackbar('Cube Thresh', 'Tuner', 100, 255, nothing)
    cv2.createTrackbar('Cube Min Area', 'Tuner', 1000, 20000, nothing)
    cv2.createTrackbar('Cube Max Area', 'Tuner', 10000, 100000, nothing)

    cv2.createTrackbar('Dimple Thresh', 'Tuner', 60, 255, nothing)
    cv2.createTrackbar('Dimple Min Area', 'Tuner', 10, 1200, nothing)
    cv2.createTrackbar('Dimple Max Area', 'Tuner', 1000, 5000, nothing)

    # Load source
    cap = None
    frame_static = None

    if isinstance(SOURCE, int) or (
            isinstance(SOURCE, str) and not any(SOURCE.lower().endswith(ext) for ext in ['.png', '.jpg', '.jpeg'])):
        cap = cv2.VideoCapture(SOURCE)
    else:
        if os.path.exists(SOURCE):
            frame_static = cv2.imread(SOURCE)
        else:
            print(f"File {SOURCE} not found.")
            return

    while True:
        if cap:
            ret, img = cap.read()
            if not ret:
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue
            current_frame = img.copy()
        else:
            current_frame = frame_static.copy()

        # Get Slider Values
        gamma_val = cv2.getTrackbarPos('Gamma', 'Tuner') / 100.0
        if gamma_val == 0: gamma_val = 0.01

        sq_tol = cv2.getTrackbarPos('Squareness', 'Tuner') / 100.0

        paper_thresh_val = cv2.getTrackbarPos('Paper Thresh', 'Tuner')
        cube_thresh_val = cv2.getTrackbarPos('Cube Thresh', 'Tuner')
        cube_min_area = cv2.getTrackbarPos('Cube Min Area', 'Tuner')
        cube_max_area = cv2.getTrackbarPos('Cube Max Area', 'Tuner')

        dimple_thresh_val = cv2.getTrackbarPos('Dimple Thresh', 'Tuner')
        dimple_min_area = cv2.getTrackbarPos('Dimple Min Area', 'Tuner')
        dimple_max_area = cv2.getTrackbarPos('Dimple Max Area', 'Tuner')

        # --- PRE-PROCESSING ---
        gamma_corrected = apply_gamma(current_frame, gamma=gamma_val)
        gray = cv2.cvtColor(gamma_corrected, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # --- DASHBOARD ---
        dashboard = [
            f"Gamma: {gamma_val:.2f}",
            f"Square Tol: {sq_tol:.2f}",
            f"Paper Thresh: {paper_thresh_val}",
            f"Cube Thresh: {cube_thresh_val}",
            f"Cube Area: {cube_min_area} - {cube_max_area}",
            f"Dimple Thresh: {dimple_thresh_val}",
            f"Dimple Area: {dimple_min_area} - {dimple_max_area}"
        ]

        # Draw background for text
        cv2.rectangle(gamma_corrected, (10, 10), (350, 220), (0, 0, 0), -1)
        y = 40
        for line in dashboard:
            cv2.putText(gamma_corrected, line, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            y += 25

        # --- 1. DETECT PAPER ---
        paper_blurred = cv2.GaussianBlur(gray, (11, 11), 0)
        _, paper_mask = cv2.threshold(paper_blurred, paper_thresh_val, 255, cv2.THRESH_BINARY)
        paper_contours, _ = cv2.findContours(paper_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        paper_contour = None
        if paper_contours:
            largest = max(paper_contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 50000:
                paper_contour = largest
                cv2.drawContours(gamma_corrected, [paper_contour], -1, (0, 255, 255), 2)

        # --- 2. FIND DIMPLES ---
        _, dimple_mask = cv2.threshold(blurred, dimple_thresh_val, 255, cv2.THRESH_BINARY_INV)
        dimple_contours, _ = cv2.findContours(dimple_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        dimple_data = []
        for d_cnt in dimple_contours:
            area = cv2.contourArea(d_cnt)
            # CHECK MAX AREA HERE
            if dimple_min_area < area < dimple_max_area:
                (x, y), radius = cv2.minEnclosingCircle(d_cnt)
                center = (int(x), int(y))
                radius = int(radius)
                dimple_data.append((center, radius))
                cv2.circle(gamma_corrected, center, radius, (255, 0, 0), 2)

        # --- 3. FIND CUBES ---
        _, cube_mask = cv2.threshold(blurred, cube_thresh_val, 255, cv2.THRESH_BINARY_INV)
        kernel = np.ones((3, 3), np.uint8)
        cube_mask = cv2.morphologyEx(cube_mask, cv2.MORPH_OPEN, kernel, iterations=2)
        cube_contours, _ = cv2.findContours(cube_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in cube_contours:
            area = cv2.contourArea(cnt)

            if cube_min_area < area < cube_max_area:
                rect = cv2.minAreaRect(cnt)
                (center_x, center_y), (width, height), angle = rect

                aspect_ratio = float(width) / height if height > 0 else 0
                if aspect_ratio < 1.0:
                    aspect_ratio = 1.0 / aspect_ratio

                is_square = aspect_ratio <= (1.0 + sq_tol)

                box = cv2.boxPoints(rect)
                box = np.int32(box)
                cx, cy = int(center_x), int(center_y)

                is_inside_paper = True
                if paper_contour is not None:
                    if cv2.pointPolygonTest(paper_contour, (cx, cy), False) < 0:
                        is_inside_paper = False

                has_dimple = False
                for (d_center, d_radius) in dimple_data:
                    if cv2.pointPolygonTest(cnt, d_center, False) > 0:
                        has_dimple = True
                        break

                if not is_inside_paper:
                    cv2.drawContours(gamma_corrected, [box], 0, (0, 0, 255), 2)
                elif not is_square:
                    cv2.drawContours(gamma_corrected, [box], 0, (128, 0, 128), 2)
                    cv2.putText(gamma_corrected, f"Rect {aspect_ratio:.1f}", (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (128, 0, 128), 2)
                elif has_dimple:
                    cv2.drawContours(gamma_corrected, [box], 0, (0, 255, 0), 2)
                    cv2.putText(gamma_corrected, "VALID", (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                else:
                    cv2.drawContours(gamma_corrected, [box], 0, (0, 165, 255), 2)
                    cv2.putText(gamma_corrected, "No Dimple", (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

        cv2.imshow('Tuner', gamma_corrected)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    if cap:
        cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    run_tuner()