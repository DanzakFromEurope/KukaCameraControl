import cv2
import numpy as np


class CalibrationSystem:
    def __init__(self):
        self.pixel_points = []  # Points from Camera (u, v)
        self.robot_points = []  # Points from Robot (X, Y)
        self.matrix = None  # The calculated Homography Matrix
        self.is_calibrated = False

    def add_point(self, pixel_pos, robot_pos):
        """
        pixel_pos: (u, v) tuple from YOLO
        robot_pos: (x, y) tuple from KUKA
        """
        self.pixel_points.append(pixel_pos)
        self.robot_points.append(robot_pos)
        print(f"🔵 Point Recorded! Pixel:{pixel_pos} -> Robot:{robot_pos}")
        print(f"   Total Points: {len(self.pixel_points)} (Need 4 for full calibration)")

    def compute_matrix(self):
        if len(self.pixel_points) < 4:
            print("⚠️ Need at least 4 points to compute accurate Homography.")
            return False

        # Convert to numpy format
        pts_src = np.array(self.pixel_points)
        pts_dst = np.array(self.robot_points)

        # Calculate Homography (Perspective Transform)
        # This maps the Plane of the Image to the Plane of the Table
        self.matrix, status = cv2.findHomography(pts_src, pts_dst)
        self.is_calibrated = True
        print("✅ Calibration Matrix Computed Successfully!")
        return True

    def pixel_to_robot(self, u, v):
        """
        Converts camera (u,v) to robot (x,y) using the matrix
        """
        if not self.is_calibrated or self.matrix is None:
            return None

        # Matrix multiplication
        # Point must be [u, v, 1]
        input_point = np.array([[[u, v]]], dtype='float32')
        output_point = cv2.perspectiveTransform(input_point, self.matrix)

        # Result is [[x, y]]
        return output_point[0][0]

    def reset(self):
        self.pixel_points = []
        self.robot_points = []
        self.is_calibrated = False
        print("Calibration Data Reset.")