import vision_detector

# --- Configuration ---
# 0 = Webcam
# 'camera_image_14.png' = Your test image
# 'video.mp4' = Your test video
SOURCE = 'test_images/camera_image_14.png'
#SOURCE = 'test_images/camera_video.mp4'


def main():
    print("Initializing Robot Control System...")

    # Run the vision system
    # This will open the window and show you the detection
    vision_detector.run_vision_system(SOURCE)

    print("System shutdown.")


if __name__ == "__main__":
    main()