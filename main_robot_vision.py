import cv2
import numpy as np
import threading
import time
import re
import math
from pynput import keyboard

from ultralytics import YOLO
import KUKA_handler
import camera_driver
import calibration_core

# --- CONFIGURATION ---
DEBUG_MODE = True  # <--- SET TO TRUE TO TEST WITHOUT ROBOT
ROBOT_IP = '192.168.1.152'
ROBOT_PORT = 7000
YOLO_MODEL = 'best.pt'

current_robot_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0}
jog_command = None
suction_active = False
blow_active = False
gripper_closed = False
app_running = True
drop_off_pos = None
auto_mode = False
robot_is_busy = False

calib_mode_active = False
calib_step = 0
calib_stored_pixels = []
calib_snapshot = None
calibration_trigger = False


def sort_cubes_reading_order(cubes):
    if len(cubes) < 2: return sorted(cubes, key=lambda c: c['cx'])
    cubes_by_y = sorted(cubes, key=lambda c: c['cy'])
    if len(cubes) == 4:
        top_row = sorted(cubes_by_y[:2], key=lambda c: c['cx'])
        bot_row = sorted(cubes_by_y[2:], key=lambda c: c['cx'])
        return top_row + bot_row
    return sorted(cubes, key=lambda c: c['cx'])


def parse_kuka_pos(pos_string):
    try:
        x_match = re.search(r'X\s+([-\d\.]+)', str(pos_string))
        y_match = re.search(r'Y\s+([-\d\.]+)', str(pos_string))
        z_match = re.search(r'Z\s+([-\d\.]+)', str(pos_string))
        x = float(x_match.group(1)) if x_match else 0.0
        y = float(y_match.group(1)) if y_match else 0.0
        z = float(z_match.group(1)) if z_match else 0.0
        return x, y, z
    except:
        pass
    return 0.0, 0.0, 0.0


def keyboard_listener():
    global jog_command, app_running, calibration_trigger, suction_active, blow_active, gripper_closed, drop_off_pos, auto_mode

    def on_press(key):
        global jog_command, calibration_trigger, blow_active, auto_mode, drop_off_pos
        try:
            if hasattr(key, 'char'):
                char = key.char.lower()
                if char == 'w':
                    jog_command = 'up'
                elif char == 's':
                    jog_command = 'down'
                elif char == 'a':
                    jog_command = 'left'
                elif char == 'd':
                    jog_command = 'right'
                elif char == 'q':
                    jog_command = 'z_up'
                elif char == 'e':
                    jog_command = 'z_down'
                elif char == 'b':
                    blow_active = True
                elif char == 'c':
                    calibration_trigger = True
                elif char == 'x':
                    drop_off_pos = (current_robot_pos['x'], current_robot_pos['y'])
                    print(f"✅ Drop-off Set: {drop_off_pos}")
                elif char == ' ':
                    if calib_mode_active:
                        print("⚠️ Cannot start Auto while Calibrating!")
                    elif drop_off_pos is None:
                        print("⚠️ Set Drop-off ('X') first!")
                    else:
                        auto_mode = not auto_mode
                        print(f"🤖 Auto Mode: {'ON' if auto_mode else 'OFF'}")
        except:
            pass

    def on_release(key):
        global jog_command, suction_active, blow_active, gripper_closed
        if hasattr(key, 'char'):
            char = key.char.lower()
            if char in ['w', 's', 'a', 'd', 'q', 'e']: jog_command = None
            if char == 'v': suction_active = not suction_active
            if char == 'b': blow_active = False
            if char == 'g': gripper_closed = not gripper_closed
        if key == keyboard.Key.esc: return False

    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()
    app_running = False


def draw_z_visualization(img_height, current_z):
    width = 150
    panel = np.zeros((img_height, width, 3), dtype=np.uint8)
    max_z = 300
    scale = img_height / max_z
    safe_y = int(img_height - (200 * scale))
    cv2.line(panel, (0, safe_y), (width, safe_y), (0, 255, 255), 1)
    cv2.putText(panel, "Safe Z", (5, safe_y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
    table_y = int(img_height - (0 * scale)) - 5
    cv2.line(panel, (0, table_y), (width, table_y), (0, 255, 0), 2)
    cv2.putText(panel, "Table", (5, table_y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
    z_pixel = int(img_height - (current_z * scale))
    z_pixel = max(10, min(img_height - 10, z_pixel))
    cv2.rectangle(panel, (40, z_pixel - 10), (110, z_pixel + 10), (255, 100, 0), -1)
    cv2.putText(panel, f"Z:{current_z:.0f}", (50, z_pixel + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return panel


def main():
    global jog_command, app_running, calibration_trigger, current_robot_pos
    global suction_active, blow_active, gripper_closed, auto_mode, robot_is_busy, drop_off_pos
    global calib_mode_active, calib_step, calib_stored_pixels, calib_snapshot

    calib = calibration_core.CalibrationSystem()

    if DEBUG_MODE:
        print("⚠️ DEBUG MODE: Using Virtual Robot")
        robot = KUKA_handler.KUKA_Mock()
        robot.KUKA_Open()
        # --- NEW: Auto-load Mock Calibration for Blue Dot ---
        calib.load_mock_calibration()
    else:
        print("🤖 Connecting to Robot...")
        try:
            robot = KUKA_handler.KUKA_Handler(ROBOT_IP, ROBOT_PORT)
            if not robot.KUKA_Open(): print("⚠️ Failed to connect.")
        except Exception as e:
            print(f"⚠️ Error: {e}")
            robot = None
        # Load real calibration if available
        calib.load()

    print("📷 Initializing Camera...")
    cam = camera_driver.CameraHandler(source="baumer")

    print("🧠 Loading AI Model...")
    try:
        model = YOLO(YOLO_MODEL)
    except:
        print(f"❌ Error: Could not load {YOLO_MODEL}")
        return

    kb_thread = threading.Thread(target=keyboard_listener, daemon=True)
    kb_thread.start()

    print("\n--- SYSTEM READY ---")
    print("  'C' Key : Starts Snapshot Calibration")

    last_jog = None
    last_suction_state = False
    last_blow_state = False
    last_gripper_state = False
    annotated_frame = None
    best_cube_robot = None
    best_cube_rot = 0
    settle_timer = 0

    while app_running:
        if robot and robot.connected:
            raw_pos = robot.KUKA_ReadVar('$POS_ACT')
            x, y, z = parse_kuka_pos(raw_pos)
            current_robot_pos['x'] = x
            current_robot_pos['y'] = y
            current_robot_pos['z'] = z

            if robot_is_busy:
                is_working = robot.KUKA_ReadVar('doPick')
                if is_working == False:
                    print("✅ Robot returned.")
                    robot_is_busy = False
                    settle_timer = time.time() + 0.5

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

        # --- VISION ---
        if calib_mode_active and calib_snapshot is not None:
            display_img = calib_snapshot.copy()
            cv2.putText(display_img, "CALIBRATION MODE", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(display_img, f"Step {calib_step + 1}/4", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255),
                        2)
            msg = f"JOG to Cube #{calib_step + 1} and PRESS 'C'"
            cv2.putText(display_img, msg, (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            if calib_step < len(calib_stored_pixels):
                target_u, target_v = calib_stored_pixels[calib_step]
                cv2.circle(display_img, (int(target_u), int(target_v)), 20, (0, 0, 255), 3)
                cv2.circle(display_img, (int(target_u), int(target_v)), 5, (0, 0, 255), -1)
            annotated_frame = display_img

        else:
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
                    best_cube_robot = None
                    min_dist = float('inf')
                    annotated_frame = frame.copy()
                    detected_cubes = []

                    for result in results:
                        obbs = result.obb
                        if obbs is not None:
                            for obb in obbs:
                                box = obb.xywhr[0].cpu().numpy()
                                cx, cy, w, h, rot = box
                                rot_deg = math.degrees(rot)
                                detected_cubes.append(
                                    {'cx': cx, 'cy': cy, 'w': w, 'h': h, 'rot': rot, 'rot_deg': rot_deg})

                    detected_cubes = sort_cubes_reading_order(detected_cubes)

                    for i, cube in enumerate(detected_cubes):
                        cx, cy = cube['cx'], cube['cy']
                        rot = cube['rot']
                        rot_deg = cube['rot_deg']
                        rect_pts = cv2.boxPoints(((cx, cy), (cube['w'], cube['h']), rot_deg))
                        cv2.drawContours(annotated_frame, [np.int32(rect_pts)], 0, (0, 255, 0), 2)
                        cv2.putText(annotated_frame, f"#{i + 1}", (int(cx) - 40, int(cy) - 40),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)

                        dist = np.sqrt((cx - screen_center[0]) ** 2 + (cy - screen_center[1]) ** 2)
                        if dist < min_dist:
                            min_dist = dist
                            best_cube_robot = None

                        if calib.is_calibrated:
                            robot_coord = calib.pixel_to_robot(cx, cy)
                            if robot_coord is not None:
                                if dist == min_dist: best_cube_robot = robot_coord
                                txt = f"X:{robot_coord[0]:.0f} Y:{robot_coord[1]:.0f}"
                                cv2.putText(annotated_frame, txt, (int(cx), int(cy) - 30), cv2.FONT_HERSHEY_SIMPLEX,
                                            0.6, (0, 255, 255), 2)

                    if auto_mode:
                        status = "MOVING" if robot_is_busy else "SCANNING"
                        cv2.putText(annotated_frame, f"STATUS: {status}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                                    (0, 255, 0), 2)
                    else:
                        cv2.putText(annotated_frame, "MODE: MANUAL", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                                    (255, 255, 0), 2)

                    if DEBUG_MODE:
                        cv2.putText(annotated_frame,
                                    f"MOCK ROBOT POS: {current_robot_pos['x']:.1f}, {current_robot_pos['y']:.1f}",
                                    (10, 560), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        if annotated_frame is None: annotated_frame = np.zeros((600, 800, 3), dtype=np.uint8)

        # --- NEW: ROBOT POS VISUALIZATION (BLUE CROSS) ---
        # Draws the simulated (or real) robot position as a Blue Crosshair on the camera image
        if calib.is_calibrated:
            pixel_pos = calib.robot_to_pixel(current_robot_pos['x'], current_robot_pos['y'])

            if pixel_pos is not None:
                pu, pv = int(pixel_pos[0]), int(pixel_pos[1])

                # Only draw if within screen bounds
                if 0 <= pu < annotated_frame.shape[1] and 0 <= pv < annotated_frame.shape[0]:
                    cv2.drawMarker(annotated_frame, (pu, pv), (255, 0, 0), cv2.MARKER_CROSS, 30, 3)
                    cv2.circle(annotated_frame, (pu, pv), 10, (255, 0, 0), 2)
                    cv2.putText(annotated_frame, "ROBOT", (pu + 15, pv), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        # --- LIVE STATUS OVERLAY (X,Y,Z) ---
        cv2.putText(annotated_frame, f"POS: X{current_robot_pos['x']:.1f} Y{current_robot_pos['y']:.1f}", (10, 560),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # --- Z-AXIS SIDE PANEL ---
        z_panel = draw_z_visualization(annotated_frame.shape[0], current_robot_pos['z'])
        final_display = np.hstack((annotated_frame, z_panel))
        cv2.imshow("Robot Vision Control", final_display)

        if calibration_trigger:
            calibration_trigger = False

            if not calib_mode_active:
                print("📸 Taking Calibration Snapshot...")
                snap_frame = cam.get_frame()
                if snap_frame is not None:
                    results = model.predict(snap_frame, verbose=False)
                    temp_cubes = []
                    for result in results:
                        if result.obb is not None:
                            for obb in result.obb:
                                box = obb.xywhr[0].cpu().numpy()
                                temp_cubes.append({'cx': box[0], 'cy': box[1]})
                    temp_cubes = sort_cubes_reading_order(temp_cubes)

                    if len(temp_cubes) < 4:
                        print(f"❌ Aborted: Found {len(temp_cubes)}/4 cubes.")
                    else:
                        calib.reset()
                        calib_stored_pixels = [(c['cx'], c['cy']) for c in temp_cubes[:4]]
                        calib_snapshot = snap_frame.copy()
                        calib_step = 0
                        calib_mode_active = True
                        print("✅ Entering Guidance Mode.")
            else:
                rob_x, rob_y = current_robot_pos['x'], current_robot_pos['y']
                pixel_u, pixel_v = calib_stored_pixels[calib_step]
                print(f"📍 Point {calib_step + 1}: Pixel({pixel_u:.0f},{pixel_v:.0f}) -> Robot({rob_x:.1f},{rob_y:.1f})")
                calib.add_point((pixel_u, pixel_v), (rob_x, rob_y))

                calib_step += 1
                if calib_step >= 4:
                    success = calib.compute_matrix()
                    if success:
                        print("🎉 CALIBRATION COMPLETE!")
                        calib_mode_active = False
                        calib_snapshot = None
                    else:
                        print("❌ Calibration Failed")
                        calib_mode_active = False

        if auto_mode and not robot_is_busy and not calib_mode_active:
            if should_grab_frame and calib.is_calibrated and drop_off_pos is not None and best_cube_robot is not None:
                target_x, target_y = best_cube_robot
                target_r = 0.0
                print(f"🚀 Target: {target_x:.1f}, {target_y:.1f}")

                pick_str = "{{X {:.2f}, Y {:.2f}, Z 5.0, A {:.2f}, B 0.0, C 180.0}}".format(target_x, target_y,
                                                                                            target_r)
                drop_str = "{{X {:.2f}, Y {:.2f}, Z 5.0, A 0.0, B 0.0, C 180.0}}".format(drop_off_pos[0],
                                                                                         drop_off_pos[1])

                robot.KUKA_WriteVar('pickPos', pick_str)
                robot.KUKA_WriteVar('dropPos', drop_str)
                robot.KUKA_WriteVar('doPick', True)
                robot_is_busy = True

        key = cv2.waitKey(1) & 0xFF
        if key == 27: app_running = False

    if robot:
        robot.KUKA_WriteVar('vacuumOn', False)
        robot.KUKA_WriteVar('blowOn', False)
        robot.KUKA_Close()

    cam.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()