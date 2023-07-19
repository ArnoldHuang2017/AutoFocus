from .Motor import NucleusNMotor
from .Handler import NucleusNHandler
import Device.SerialComm as SerialComm


def Start():
    print("Call NucleusNHandler().Start()")

    SerialComm.Start()
    NucleusNHandler().Start()


def CalibrateMotor():
    NucleusNMotor().Calibrate()


def MoveMotor(position):
    NucleusNMotor().Move(position)


def SetCALButtonShortPressHandler(callback):
    NucleusNHandler().SetCALButtonShortPressHandler(callback)


def SetCALButtonLongPressHandler(callback):
    NucleusNHandler().SetCALButtonLongPressHandler(callback)


def SetRecordOnButtonLongPressHandler(callback):
    NucleusNHandler().SetRecordOnButtonHandler(callback)


def SetRecordOffButtonLongPressHandler(callback):
    NucleusNHandler().SetRecordOffButtonHandler(callback)


def SetWheelHandler(callback):
    NucleusNHandler().SetWheelHandler(callback)


def WriteRawDataToMotor(data):
    NucleusNMotor().WriteRawData(data)


def Stop():
    SerialComm.Stop()


__all__ = ["Start", "MoveMotor", "Stop", "SetCALButtonShortPressHandler", "SetCALButtonLongPressHandler",
           "CalibrateMotor", "SetRecordOffButtonLongPressHandler", "SetRecordOnButtonLongPressHandler",
           "SetWheelHandler", "WriteRawDataToMotor"]
