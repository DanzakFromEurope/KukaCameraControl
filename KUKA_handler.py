from py_openshowvar import openshowvar
import time

class KUKA_Handler:
    """
    Class implementing openshowvar and making it more user-friendly
    """
    def __init__(self, ipAddress, port):
        self.connected = False
        self.ipAddress = ipAddress
        self.port = port
        self.client = None

    def KUKA_Open(self):
        if self.connected == False:
            self.client = openshowvar(self.ipAddress, self.port)
            res = self.client.can_connect

            if res == True:
                print('Connection is established!')
                self.connected = True
                return True
            else:
                print('Connection is broken! Check configuration or restart C3_Server at KUKA side.')
                self.connected = False
                return False
        else:
            print('Connection is ready!')

    def KUKA_ReadVar(self, var):
        if self.connected:
            res = self.client.read(var, debug=False)
            if res == b'TRUE':
                return True
            elif res == b'FALSE':
                return False
            else:
                return res
        else:
            return False

    def KUKA_WriteVar(self, var, value):
        if self.connected:
            self.client.write(var, str(value))
            return True
        else:
            return False

    def KUKA_Close(self):
        if self.connected == True:
            self.client.close()
            self.connected = False
            return True

        else:
            return False



# # Use of openshowvar without KUKA_Handler
# robot = openshowvar('192.168.1.152',7000)
#
# # program_speed = robot.read('$OV_PRO',debug=True)
# # print(int(program_speed.decode())) # reading the value of system variable, decoding it to string and retyping to integer
# #
# # new_advance = '3'
# # robot.write('$ADVANCE',new_advance,debug=True) # Writing new value to the system variable, must be type string
#
# program_speed =robot.read('pyposition',debug=False)
# print(program_speed.decode()) # Reading a value of a global variable, must be declared in $config.dat
#
# value = '{POS: X 7.38167620, Y -359.959778, Z 418.508301, A 90.0001526, B 29.9999981, C 179.999863}'
# robot.write('pyposition',value,debug=False)  # Writing new value to the global variable, must be declared in $config.dat
#
# robot.close()  #closing connection with openshowvar
# exit()
# ###############################
#
#
# robot = KUKA_Handler('192.168.1.152', 7000) # Using pyopenshowvar with the help of KUKA_Handler
# robot.KUKA_Open() # opening connection
# value_to_send = 5
# robot.KUKA_WriteVar('My_global_variable', value_to_send) # Writing a global variable to robot, it must be declared in $config.dat
# for i in range(0,30):
#     recieved_value = robot.KUKA_ReadVar('$POS_ACT') # reading system variable every one second
#     print(recieved_value)
#     time.sleep(1)
# robot.KUKA_Close()
