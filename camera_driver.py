import cv2
import numpy as np
import sys

# Try importing neoapi
try:
    import neoapi

    NEOAPI_AVAILABLE = True
except ImportError:
    NEOAPI_AVAILABLE = False


class CameraHandler:
    def __init__(self, source="baumer"):
        self.source = source
        self.camera = None
        self.cap = None
        self.use_simulation = False

        if source == "baumer":
            if NEOAPI_AVAILABLE:
                try:
                    self.camera = neoapi.Cam()
                    self.camera.Connect()
                    self.camera.f.ExposureTime.Set(10000)

                    # FIX: Force the camera to send Mono8 (Grayscale) to be consistent
                    # If your camera supports Color, you can change this to 'BGR8'
                    if hasattr(self.camera.f, 'PixelFormat'):
                        self.camera.f.PixelFormat.SetString('Mono8')

                    print("✅ Baumer Camera Connected.")
                except Exception as e:
                    print(f"⚠️ Could not connect to Baumer Camera: {e}")
                    print("   -> Falling back to Webcam/Simulation.")
                    self.use_simulation = True
            else:
                print("⚠️ 'neoapi' library not found. Falling back to Simulation.")
                self.use_simulation = True

        if self.use_simulation or source != "baumer":
            self.cap = cv2.VideoCapture(0)

    def get_frame(self):
        """
        Returns a numpy image (BGR format) ready for OpenCV/YOLO.
        """
        if not self.use_simulation and self.camera.IsConnected():
            try:
                image = self.camera.GetImage()
                img_np = image.GetNPArray()

                # --- SHAPE HANDLING FIX ---
                # Case 1: Shape is (H, W) -> Simple Grayscale
                if len(img_np.shape) == 2:
                    img_np = cv2.cvtColor(img_np, cv2.COLOR_GRAY2BGR)

                # Case 2: Shape is (H, W, 1) -> Grayscale with explicit channel dim
                elif len(img_np.shape) == 3 and img_np.shape[2] == 1:
                    # Squeeze the last dimension to make it (H, W), then convert
                    img_np = cv2.cvtColor(img_np.squeeze(-1), cv2.COLOR_GRAY2BGR)

                # Case 3: Shape is (H, W, 3) -> Already Color (BGR or RGB)
                # No action needed usually, unless Bayer pattern

                return img_np

            except Exception as e:
                print(f"Error grabbing Baumer frame: {e}")
                return None

        elif self.cap:
            ret, frame = self.cap.read()
            if not ret:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = self.cap.read()

            if frame is not None:
                # Simulation: Color -> Gray -> BGR to match Industrial look
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

            return frame

        return None

    def release(self):
        if self.cap:
            self.cap.release()