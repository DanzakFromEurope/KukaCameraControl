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

                    # --- FIX 1: Force Mono8 (Grayscale) ---
                    if hasattr(self.camera.f, 'PixelFormat'):
                        self.camera.f.PixelFormat.SetString('Mono8')

                    # This stops the camera from streaming video.
                    # It will only take a photo when we call .Execute()
                    if hasattr(self.camera.f, 'TriggerMode'):
                        self.camera.f.TriggerMode.SetString('On')
                        self.camera.f.TriggerSource.SetString('Software')

                    print("✅ Baumer Camera Connected (Software Trigger Mode).")
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
                # Tell the camera to snap the photo NOW
                if hasattr(self.camera.f, 'TriggerSoftware'):
                    self.camera.f.TriggerSoftware.Execute()

                # Wait for the image to arrive (Timeout default is usually 1000ms)
                image = self.camera.GetImage()
                img_np = image.GetNPArray()

                # --- SHAPE HANDLING ---
                # Case 1: (H, W) -> Grayscale
                if len(img_np.shape) == 2:
                    img_np = cv2.cvtColor(img_np, cv2.COLOR_GRAY2BGR)

                # Case 2: (H, W, 1) -> Grayscale with channel dim
                elif len(img_np.shape) == 3 and img_np.shape[2] == 1:
                    img_np = cv2.cvtColor(img_np.squeeze(-1), cv2.COLOR_GRAY2BGR)

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
                # Simulation: Color -> Gray -> BGR
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

            return frame

        return None

    def release(self):
        if self.cap:
            self.cap.release()