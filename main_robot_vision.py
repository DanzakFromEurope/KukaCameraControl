import cv2
import numpy as np
import threading
import time
import re
import math
from pynput import keyboard

# Import our modules
from ultralytics import YOLO
import KUKA_handler
import camera_driver
import calibration_core

# --- CONFIGURATION ---
ROBOT_IP = '192.168.1.152'  # Corrected IP
ROBOT_PORT = 7000
YOLO_MODEL = 'best.pt'

# --- GLOBAL STATE ---
current_robot_pos = {'x': 0.0, 'y': 0.0}
jog_command = None

# Tool States
suction_active = False
blow_active = False
gripper_closed = False

app_running = True
calibration_trigger = False
drop_off_pos = None
auto_mode = False
robot_is_busy = False


# --- KUKA HELPER FUNCTIONS ---
def parse_kuka_pos(pos_string):
    try:
        x_match = re.search(r'X\s+([-\d\.]+)', str(pos_string))
        y_match = re.search(r'Y\s+([-\d\.]+)', str(pos_string))
        if x_match and y_match:
            return float(x_match.group(1)), float(y_match.group(1))
    except Exception as e:
        print(f"Error parsing position: {e}")
    return 0.0, 0.0


# --- KEYBOARD THREAD ---
def keyboard_listener():
    global jog_command, app_running, calibration_trigger
    global suction_active, blow_active, gripper_closed
    global drop_off_pos, auto_mode

    def on_press(key):
        global jog_command, calibration_trigger, blow_active, auto_mode, drop_off_pos
        try:
            # XY Movement
            if key == keyboard.Key.up:
                jog_command = 'up'
            elif key == keyboard.Key.down:
                jog_command = 'down'
            elif key == keyboard.Key.left:
                jog_command = 'left'
            elif key == keyboard.Key.right:
                jog_command = 'right'
            # Z Movement
            elif key == keyboard.Key.page_up:
                jog_command = 'z_up'
            elif key == keyboard.Key.page_down:
                jog_command = 'z_down'

            # Functional Keys
            elif hasattr(key, 'char'):
                if key.char == 'c': calibration_trigger = True
                if key.char == 'q': return False
                if key.char == 'b': blow_active = True

                if key.char == 'd':
                    drop_off_pos = (current_robot_pos['x'], current_robot_pos['y'])
                    print(f"✅ Drop-off Set: {drop_off_pos}")

                if key.char == 'a':
                    if drop_off_pos is None:
                        print("⚠️ Cannot start Auto Mode: Set Drop-off ('D') first!")
                    else:
                        auto_mode = not auto_mode
                        print(f"🤖 Auto Mode: {'ON' if auto_mode else 'OFF'}")

        except:
            pass

    def on_release(key):
        global jog_command, suction_active, blow_active, gripper_closed

        if key in [keyboard.Key.up, keyboard.Key.down, keyboard.Key.left, keyboard.Key.right,
                   keyboard.Key.page_up, keyboard.Key.page_down]:
            jog_command = None

        if key == keyboard.Key.esc:
            return False

        if hasattr(key, 'char'):
            if key.char == 's': suction_active = not suction_active
            if key.char == 'b': blow_active = False
            if key.char == 'g': gripper_closed = not gripper_closed

    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

    app_running = False


# --- MAIN APPLICATION ---
def main():
    global jog_command, app_running, calibration_trigger, current_robot_pos
    global suction_active, blow_active, gripper_closed
    global auto_mode, robot_is_busy, drop_off_pos

    print("🤖 Connecting to Robot...")
    try:
        robot = KUKA_handler.KUKA_Handler(ROBOT_IP, ROBOT_PORT)
        if not robot.KUKA_Open():
            print("⚠️ Failed to connect to Robot (Simulation Mode?)")
    except Exception as e:
        print(f"⚠️ Robot Connection Error: {e}")
        robot = None

    print("📷 Initializing Camera...")
    cam = camera_driver.CameraHandler(source="baumer")

    print("🧠 Loading AI Model...")
    try:
        model = YOLO(YOLO_MODEL)
    except:
        print(f"❌ Error: Could not load {YOLO_MODEL}")
        return

    calib = calibration_core.CalibrationSystem()

    kb_thread = threading.Thread(target=keyboard_listener, daemon=True)
    kb_thread.start()

    print("\n--- SYSTEM READY ---")
    print("  ARROWS    : Jog X/Y")
    print("  PgUp/PgDn : Jog Z (Height)")
    print("  'D'       : Set DROP-OFF Location")
    print("  'A'       : Start/Stop AUTO MODE")
    print("  'C'       : Record Calibration Point")
    print("  'M'       : Calculate Matrix")
    print("  'Q'       : Quit")

    last_jog = None
    last_suction_state = False
    last_blow_state = False
    last_gripper_state = False

    # Variables for the Snapshot Logic
    annotated_frame = None
    best_cube_pixel = None
    best_cube_robot = None
    best_cube_rot = 0

    settle_timer = 0

    while app_running:
        # --- 1. READ ROBOT STATUS ---
        if robot and robot.connected:
            raw_pos = robot.KUKA_ReadVar('$POS_ACT')
            x, y = parse_kuka_pos(raw_pos)
            current_robot_pos['x'] = x
            current_robot_pos['y'] = y

            if robot_is_busy:
                is_working = robot.KUKA_ReadVar('doPick')
                if is_working == False:
                    print("✅ Robot returned to home/drop position.")
                    robot_is_busy = False
                    settle_timer = time.time() + 0.5

                    # --- 2. HANDLE MANUAL JOGGING ---
        if robot and robot.connected and not auto_mode:
            if suction_active != last_suction_state:
                robot.KUKA_WriteVar('vacuumOn', suction_active)
                last_suction_state = suction_active
            if blow_active != last_blow_state:
                robot.KUKA_WriteVar('blowOn', blow_active)
                last_blow_state = blow_active
            if gripper_closed != last_gripper_state:
                robot.KUKA_WriteVar('gripperClose', gripper_closed)
                last_gripper_state = gripper_closed

            if jog_command != last_jog:
                robot.KUKA_WriteVar('goUp', False)
                robot.KUKA_WriteVar('goDown', False)
                robot.KUKA_WriteVar('goLeft', False)
                robot.KUKA_WriteVar('goRight', False)
                robot.KUKA_WriteVar('goZUp', False)
                robot.KUKA_WriteVar('goZDown', False)

                if jog_command == 'up':
                    robot.KUKA_WriteVar('goUp', True)
                elif jog_command == 'down':
                    robot.KUKA_WriteVar('goDown', True)
                elif jog_command == 'left':
                    robot.KUKA_WriteVar('goLeft', True)
                elif jog_command == 'right':
                    robot.KUKA_WriteVar('goRight', True)
                elif jog_command == 'z_up':
                    robot.KUKA_WriteVar('goZUp', True)
                elif jog_command == 'z_down':
                    robot.KUKA_WriteVar('goZDown', True)

                last_jog = jog_command

        # --- 3. VISION LOGIC ---
        should_grab_frame = False
        if not auto_mode:
            should_grab_frame = True
        elif auto_mode and not robot_is_busy and time.time() > settle_timer:
            should_grab_frame = True

        if should_grab_frame:
            frame = cam.get_frame()
            if frame is not None:
                results = model.predict(frame, verbose=False)

                height, width = frame.shape[:2]
                screen_center = (width // 2, height // 2)
                best_cube_pixel = None
                best_cube_robot = None
                min_dist = float('inf')

                annotated_frame = frame.copy()

                for result in results:
                    obbs = result.obb
                    if obbs is not None:
                        for obb in obbs:
                            box = obb.xywhr[0].cpu().numpy()
                            cx, cy, w, h, rot = box  # rot is in Radians
                            rot_deg = math.degrees(rot)

                            # Draw Box
                            rect_pts = cv2.boxPoints(((cx, cy), (w, h), rot_deg))
                            rect_pts = np.int32(rect_pts)
                            cv2.drawContours(annotated_frame, [rect_pts], 0, (0, 255, 0), 2)

                            # Draw Center
                            cv2.circle(annotated_frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)

                            # --- VISUALIZATION RESTORED ---
                            # Draw Orientation Line (Blue)
                            line_len = 40
                            end_x = int(cx + line_len * math.cos(rot))
                            end_y = int(cy + line_len * math.sin(rot))
                            cv2.line(annotated_frame, (int(cx), int(cy)), (end_x, end_y), (255, 0, 0), 2)

                            # Draw Angle Text
                            angle_text = f"{int(rot_deg)} deg"
                            cv2.putText(annotated_frame, angle_text, (int(cx), int(cy)),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

                            # Find Best Cube Logic
                            dist = np.sqrt((cx - screen_center[0]) ** 2 + (cy - screen_center[1]) ** 2)
                            if dist < min_dist:
                                min_dist = dist
                                best_cube_pixel = (cx, cy)
                                best_cube_rot = rot_deg

                            # Draw Robot Coords (Cyan) if Calibrated
                            if calib.is_calibrated:
                                robot_coord = calib.pixel_to_robot(cx, cy)
                                if robot_coord is not None:
                                    if dist == min_dist:
                                        best_cube_robot = robot_coord
                                    coord_text = f"X:{robot_coord[0]:.0f} Y:{robot_coord[1]:.0f}"
                                    cv2.putText(annotated_frame, coord_text, (int(cx), int(cy) - 30),
                                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                # Center Crosshair
                cv2.line(annotated_frame, (screen_center[0] - 20, screen_center[1]),
                         (screen_center[0] + 20, screen_center[1]), (0, 0, 255), 1)
                cv2.line(annotated_frame, (screen_center[0], screen_center[1] - 20),
                         (screen_center[0], screen_center[1] + 20), (0, 0, 255), 1)

        # --- 4. DISPLAY ---
        if annotated_frame is None:
            annotated_frame = np.zeros((600, 800, 3), dtype=np.uint8)

        display_img = annotated_frame.copy()

        if auto_mode:
            if robot_is_busy:
                cv2.putText(display_img, "STATUS: ROBOT MOVING", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            else:
                cv2.putText(display_img, "STATUS: SCANNING...", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            cv2.putText(display_img, "MODE: MANUAL", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

        if drop_off_pos:
            cv2.putText(display_img, "DROP-OFF SET", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

        # --- 5. AUTO LOGIC ---
        if auto_mode and robot and robot.connected and not robot_is_busy:
            if should_grab_frame and calib.is_calibrated and drop_off_pos is not None:
                if best_cube_robot is not None:
                    target_x, target_y = best_cube_robot
                    target_r = best_cube_rot

                    print(f"🚀 Target Found: {target_x:.1f}, {target_y:.1f}, Angle {target_r:.1f}")

                    pick_str = "{{X {:.2f}, Y {:.2f}, Z 0.0, A {:.2f}, B 0.0, C 180.0}}".format(
                        target_x, target_y, target_r
                    )
                    drop_str = "{{X {:.2f}, Y {:.2f}, Z 0.0, A 0.0, B 0.0, C 180.0}}".format(
                        drop_off_pos[0], drop_off_pos[1]
                    )

                    robot.KUKA_WriteVar('pickPos', pick_str)
                    robot.KUKA_WriteVar('dropPos', drop_str)
                    robot.KUKA_WriteVar('doPick', True)

                    robot_is_busy = True
                else:
                    print("... No cubes detected.")

        # --- 6. CALIBRATION ---
        if calibration_trigger:
            calibration_trigger = False
            if best_cube_pixel is None:
                print("⚠️ No cube detected!")
            elif robot is None or not robot.connected:
                print("⚠️ Robot not connected!")
            else:
                calib.add_point(best_cube_pixel, (current_robot_pos['x'], current_robot_pos['y']))

        cv2.imshow("Robot Vision Control", display_img)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('m'):
            calib.compute_matrix()
        elif key == ord('r'):
            calib.reset()
        elif key == ord('q'):
            app_running = False

    if robot:
        robot.KUKA_WriteVar('vacuumOn', False)
        robot.KUKA_WriteVar('blowOn', False)
        robot.KUKA_Close()

    cam.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()