from py_openshowvar import openshowvar
from pynput import keyboard
import KUKA_handler
import sys

try:
    robot = KUKA_handler.KUKA_Handler('192.168.1.153', 7000)
    print("Attempting to connect to KUKA robot...")
    robot.KUKA_Open()
    print("Connection established.")
except Exception as e:
    print(f"Error connecting to KUKA robot: {e}")
    sys.exit(1)  # Exit if connection fails

def on_press(key):
    try:
        if key == keyboard.Key.up:
            print("Up Arrow Pressed")
            robot.KUKA_WriteVar('goUp', True)
        elif key == keyboard.Key.down:
            print("Down Arrow Pressed")
            robot.KUKA_WriteVar('goDown', True)
        elif key == keyboard.Key.left:
            print("Left Arrow Pressed")
            robot.KUKA_WriteVar('goLeft', True)
        elif key == keyboard.Key.right:
            print("Right Arrow Pressed")
            robot.KUKA_WriteVar('goRight', True)

        # Check for the Escape key to stop the listener and close the connection
        elif key == keyboard.Key.esc:
            print("\nscape key pressed. Stopping listener and closing connection.")
            return False  # Stops the listener

    except AttributeError:
        # Ignore regular alphanumeric key presses
        pass
    except Exception as e:
        print(f"An error occurred during key processing: {e}")
        # Stop the listener on unexpected error
        return False

def on_release(key):
    try:
        if key == keyboard.Key.up:
            print("⬆️ Up Arrow Released (STOP movement)")
            print("Position: " + str(robot.KUKA_ReadVar('$POS_ACT'))) # Read position after movement stops
            robot.KUKA_WriteVar('goUp', False)
        elif key == keyboard.Key.down:
            print("⬇️ Down Arrow Released (STOP movement)")
            print("Position: " + str(robot.KUKA_ReadVar('$POS_ACT')))
            robot.KUKA_WriteVar('goDown', False)
        elif key == keyboard.Key.left:
            print("⬅️ Left Arrow Released (STOP movement)")
            print("Position: " + str(robot.KUKA_ReadVar('$POS_ACT')))
            robot.KUKA_WriteVar('goLeft', False)
        elif key == keyboard.Key.right:
            print("➡️ Right Arrow Released (STOP movement)")
            print("Position: " + str(robot.KUKA_ReadVar('$POS_ACT')))
            robot.KUKA_WriteVar('goRight', False)
    except AttributeError:
        # Ignore regular alphanumeric key presses
        pass
    except Exception as e:
        print(f"An error occurred during key processing: {e}")
        # Stop the listener on unexpected error
        return False

def start_arrow_listener():
    print("\n🚀 KUKA Robot Controller Active.")
    print("Press **Up, Down, Left, or Right** to jog the robot.")
    print("Press **Esc** to exit the program and close the connection.")
    print("-" * 50)

    listener = keyboard.Listener(
        on_press=on_press,
        on_release=on_release
    )
    listener.start()
    listener.join()

    print("-" * 50)
    print("Listener stopped. Proceeding to close KUKA connection.")


if __name__ == "__main__":
    try:
        start_arrow_listener()

    except KeyboardInterrupt:
        print("\nProgram interrupted by user (Ctrl+C).")

    finally:
        # --- 4. Robot Disconnection ---
        print("Closing KUKA connection.")
        robot.KUKA_Close()
        print("Program finished.")
        sys.exit(0)