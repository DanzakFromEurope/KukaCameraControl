import cv2
import numpy as np
import os


class CalibrationSystem:
    def __init__(self):
        self.pixel_points = []
        self.robot_points = []
        self.matrix = None
        self.matrix_inv = None  # NEW: Inverse Matrix for Robot->Pixel
        self.is_calibrated = False
        self.filename = "calibration_matrix.npy"

    def add_point(self, pixel_pos, robot_pos):
        self.pixel_points.append(pixel_pos)
        self.robot_points.append(robot_pos)
        print(f"🔵 Point Recorded! Pixel:{pixel_pos} -> Robot:{robot_pos}")

    def compute_matrix(self):
        if len(self.pixel_points) < 4:
            print("⚠️ Need at least 4 points.")
            return False

        pts_src = np.array(self.pixel_points)
        pts_dst = np.array(self.robot_points)

        if np.var(pts_dst[:, 0]) == 0 and np.var(pts_dst[:, 1]) == 0:
            print("❌ Calibration Failed: All robot points are the same.")
            return False

        try:
            # Forward Matrix (Pixel -> Robot)
            self.matrix, status = cv2.findHomography(pts_src, pts_dst)

            # NEW: Inverse Matrix (Robot -> Pixel)
            # We simply invert the forward matrix
            self.matrix_inv = np.linalg.inv(self.matrix)

        except Exception as e:
            print(f"❌ Matrix Calculation Error: {e}")
            return False

        if self.matrix is None:
            return False

        self.is_calibrated = True
        print("✅ Calibration Matrix Computed Successfully!")
        self.save()
        return True

    def pixel_to_robot(self, u, v):
        if not self.is_calibrated or self.matrix is None: return None
        input_point = np.array([[[u, v]]], dtype='float32')
        output_point = cv2.perspectiveTransform(input_point, self.matrix)
        return output_point[0][0]

    # NEW FUNCTION
    def robot_to_pixel(self, x, y):
        """
        Converts Robot (x,y) back to Pixel (u,v) for visualization.
        """
        if not self.is_calibrated or self.matrix_inv is None: return None

        input_point = np.array([[[x, y]]], dtype='float32')
        output_point = cv2.perspectiveTransform(input_point, self.matrix_inv)

        return output_point[0][0]

    def reset(self):
        self.pixel_points = []
        self.robot_points = []
        self.is_calibrated = False
        self.matrix = None
        self.matrix_inv = None
        print("Calibration Data Reset.")

    def save(self):
        if self.matrix is not None:
            np.save(self.filename, self.matrix)
            print(f"💾 Calibration saved to '{self.filename}'")

    def load(self):
        if os.path.exists(self.filename):
            try:
                self.matrix = np.load(self.filename)
                # Calculate inverse immediately on load
                self.matrix_inv = np.linalg.inv(self.matrix)
                self.is_calibrated = True
                print(f"📂 Calibration loaded from '{self.filename}'")
                return True
            except Exception as e:
                print(f"⚠️ Failed to load calibration: {e}")
        else:
            print("ℹ️ No existing calibration file found. Please calibrate.")
        return False