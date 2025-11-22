from py_openshowvar import openshowvar
import time
import socket


class KUKA_Handler:
    def __init__(self, ipAddress, port):
        self.connected = False
        self.ipAddress = ipAddress
        self.port = port
        self.client = None

    def KUKA_Open(self):
        if self.connected == False:
            original_timeout = socket.getdefaulttimeout()
            socket.setdefaulttimeout(5.0)
            try:
                print(f"Attempting to connect to {self.ipAddress}:{self.port}...")
                self.client = openshowvar(self.ipAddress, self.port)
                res = self.client.can_connect
                if res == True:
                    print('Connection is established!')
                    self.connected = True
                    return True
                else:
                    self.connected = False
                    return False
            except Exception as e:
                print(f"❌ Connection Failed: {e}")
                self.connected = False
                return False
            finally:
                socket.setdefaulttimeout(original_timeout)
        else:
            print('Connection is ready!')

    def KUKA_ReadVar(self, var):
        if self.connected:
            try:
                res = self.client.read(var, debug=False)
                if res == b'TRUE':
                    return True
                elif res == b'FALSE':
                    return False
                else:
                    return res
            except:
                self.connected = False
                return False
        return False

    def KUKA_WriteVar(self, var, value):
        if self.connected:
            try:
                self.client.write(var, str(value))
                return True
            except:
                self.connected = False
                return False
        return False

    def KUKA_Close(self):
        if self.connected:
            try:
                self.client.close()
            except:
                pass
            self.connected = False
            return True
        return False


# --- NEW: DEBUG MOCK CLASS ---
class KUKA_Mock:
    """
    Simulates a KUKA robot for testing without hardware.
    """

    def __init__(self):
        self.connected = False
        self.vars = {
            'goUp': 'FALSE', 'goDown': 'FALSE', 'goLeft': 'FALSE', 'goRight': 'FALSE',
            'goZUp': 'FALSE', 'goZDown': 'FALSE',
            'vacuumOn': 'FALSE', 'blowOn': 'FALSE', 'gripperClose': 'FALSE',
            'doPick': 'FALSE',
            # Virtual Position (Starts at 0,0,100)
            'x': 0.0, 'y': 0.0, 'z': 100.0
        }
        self.last_update = time.time()

    def KUKA_Open(self):
        print("⚠️ STARTED IN DEBUG/MOCK MODE (No Real Robot)")
        self.connected = True
        return True

    def KUKA_ReadVar(self, var):
        # Simulate Movement Physics
        now = time.time()
        dt = now - self.last_update
        self.last_update = now
        speed = 50.0 * dt  # 50mm/sec simulation speed

        if self.vars['goUp'] == 'TRUE': self.vars['x'] += speed
        if self.vars['goDown'] == 'TRUE': self.vars['x'] -= speed
        if self.vars['goLeft'] == 'TRUE': self.vars['y'] += speed
        if self.vars['goRight'] == 'TRUE': self.vars['y'] -= speed

        # --- Z AXIS SIMULATION ---
        if self.vars['goZUp'] == 'TRUE': self.vars['z'] += speed
        if self.vars['goZDown'] == 'TRUE': self.vars['z'] -= speed

        if var == '$POS_ACT':
            return f"{{E6POS: X {self.vars['x']:.2f}, Y {self.vars['y']:.2f}, Z {self.vars['z']:.2f}, A 0, B 0, C 0}}"

        val = self.vars.get(var, 'FALSE')
        if val == 'TRUE': return True
        if val == 'FALSE': return False
        return val

    def KUKA_WriteVar(self, var, value):
        str_val = 'TRUE' if value is True else 'FALSE' if value is False else str(value)
        self.vars[var] = str_val

        if var == 'doPick' and value is True:
            print("   [MOCK] Received Pick Command... Simulating work...")
            time.sleep(1.0)
            self.vars['doPick'] = 'FALSE'
            print("   [MOCK] Task Finished.")

        return True

    def KUKA_Close(self):
        self.connected = False
        return True