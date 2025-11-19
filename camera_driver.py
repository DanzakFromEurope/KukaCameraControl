import cv2
import numpy as np
import sys

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
                    self.camera.f.ExposureTime.Set(10000)  # Set exposure (adjustable)
                    print("✅ Baumer Camera Connected.")
                except Exception as e:
                    print(f"⚠️ Could not connect to Baumer Camera: {e}")
                    print("   -> Falling back to Webcam/Simulation.")
                    self.use_simulation = True
            else:
                print("⚠️ 'neoapi' library not found. Falling back to Simulation.")
                self.use_simulation = True

        if self.use_simulation or source != "baumer":
            # Fallback to Webcam (0) or a file
            self.cap = cv2.VideoCapture(0)
            # If you want to test with a file, un-comment below:
            # self.cap = cv2.VideoCapture('camera_image_14.png')

    def get_frame(self):
        """
        Returns a numpy image (BGR format) ready for OpenCV/YOLO.
        """
        if not self.use_simulation and self.camera.IsConnected():
            try:
                # Get image from Baumer
                image = self.camera.GetImage()
                # Convert to numpy array
                img_np = image.GetNPArray()

                # Baumer usually gives Mono8 (Grayscale) or Bayer.
                # If Mono8, convert to BGR for YOLO
                if len(img_np.shape) == 2:
                    img_np = cv2.cvtColor(img_np, cv2.COLOR_GRAY2BGR)
                else:
                    # If BayerRG8, convert (adjust code based on your specific camera format)
                    # img_np = cv2.cvtColor(img_np, cv2.COLOR_BayerBG2BGR)
                    pass
                return img_np
            except Exception as e:
                print(f"Error grabbing Baumer frame: {e}")
                return None

        elif self.cap:
            # Webcam / File simulation
            ret, frame = self.cap.read()
            if not ret:
                # Loop video if it ends
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = self.cap.read()
            return frame

        return None

    def release(self):
        if self.camera:
            pass  # NeoAPI handles cleanup mostly, can add Disconnect() here
        if self.cap:
            self.cap.release()