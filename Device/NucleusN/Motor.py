from Device import SerialComm
from Common import *

@singleton
class NucleusNMotor(object):
    HEADER_BYTE = chr(0x3a)
    TAIL_BYTE = chr(0x0D)
    MOTOR_ID_H = '0'
    MOTOR_ID_L = '1'
    COMMAND_H = '0'
    COMMAND_L = '6'
    PARAM_H_H = '0'
    PARAM_H_L = '6'
    PARAM_L_H = '0'
    PARAM_L_L = '0'

    def __init__(self):
        pass

    def Start(self):
        pass

    def Move(self, position):
        position_h = position >> 8
        position_l = position & 0x00ff

        position_h_h, position_h_l = Hex2Ascii(position_h)
        position_l_h, position_l_l = Hex2Ascii(position_l)

        command = Ascii2Hex(self.COMMAND_H, self.COMMAND_L)
        param_h = Ascii2Hex(self.PARAM_H_H, self.PARAM_H_L)
        param_l = Ascii2Hex(self.PARAM_L_H, self.PARAM_L_L)

        checksum = 255 - (command + param_h + param_l + position_h + position_l)
        checksum_hex = checksum & 0xff

        checksum_h, checksum_l = Hex2Ascii(checksum_hex)
        message = [self.HEADER_BYTE,
                   self.MOTOR_ID_H, self.MOTOR_ID_L,  # '0', '1', 01 means motor index 01
                   self.COMMAND_H, self.COMMAND_L,  # '0', '6', 06 means command
                   self.PARAM_H_H, self.PARAM_H_L, self.PARAM_L_H, self.PARAM_L_L,  # 0600, parameter
                   position_h_h, position_h_l, position_l_h, position_l_l,  # data
                   checksum_h, checksum_l,  # checksum
                   self.TAIL_BYTE]

        msg = "".join(message) + "\n"
        SerialComm.Write(msg)

    def Calibrate(self):
        """
        CAL long: :3F0600010001B9
        :return:
        """
        print("Calibrating the motor....")
        command = ":3F0600010001B9\r\n"
        SerialComm.Write(command)

    def WriteRawData(self, data):
        SerialComm.Write(data)

    def ReadResponse(self):
        data = SerialComm.Read()
        print(data)
        return data
