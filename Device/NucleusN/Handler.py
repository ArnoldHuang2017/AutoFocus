# -- coding: utf-8 -
from Device.SerialComm import *
from Common import *
import re


@singleton
class NucleusNHandler(object):
    def __init__(self):
        self._thread = None
        self._rec_on_event_callback = None
        self._rec_off_event_callback = None
        self._cal_short_press_event_callback = None
        self._cal_long_press_event_callback = None
        self._wheel_event_callback = None
        self.motor_position = 9999

    def Start(self):
        self._thread = ThreadingRead(read_callback=self.SerialEventHandler)
        print("Start to monitor the serial reading..")

    def SerialEventHandler(self, data):
        """
        Rec ON: :C9060000000031
        Rec OFF: :C9060000000130
        CAL short: :960604D200048A :960604D200008E
        CAL long: :3F0600010001B9
        Wheel: From :0106060003F4FC to :010606000DE9FD
        """
        res = re.search(":(?P<cmd>\S\S)\S+\r", data)
        if res:
            pattern_value = res.groupdict()
            command = pattern_value.get("cmd")
            if command == "C9":
                rec_on = res[0][12]
                if rec_on == "1":
                    if self._rec_on_event_callback:
                        self._rec_on_event_callback()
                else:
                    if self._rec_off_event_callback:
                        self._rec_off_event_callback()
            elif command == "96":
                if self._cal_short_press_event_callback:
                    self._cal_short_press_event_callback()
            elif command == "3F":
                if self._cal_long_press_event_callback:
                    self._cal_long_press_event_callback()
            elif command == "01":
                num_1 = res[0][9]
                num_2 = res[0][10]
                num_3 = res[0][11]
                num_4 = res[0][12]
                position_string =  f"{num_1}{num_2}{num_3}{num_4}"
                try:
                    motor_position = int(position_string, 16)
                    if self._wheel_event_callback:
                        self._wheel_event_callback(res[0], motor_position)
                except:
                    pass

            else:
                print(f"Unknown message: {res[0]}")

        return False

    def SetRecordOnButtonHandler(self, callback):
        self._rec_on_event_callback = callback

    def ClearRecordOnButtonHandler(self):
        self._rec_on_event_callback = None

    def SetRecordOffButtonHandler(self, callback):
        self._rec_off_event_callback = callback

    def ClearRecordOffButtonHandler(self):
        self._rec_off_event_callback = None

    def SetCALButtonShortPressHandler(self, callback):
        self._cal_short_press_event_callback = callback

    def ClearCALButtonShortPressHandler(self):
        self._cal_short_press_event_callback = None

    def SetCALButtonLongPressHandler(self, callback):
        self._cal_long_press_event_callback = callback

    def ClearCALButtonLongPressHandler(self):
        self._cal_long_press_event_callback = None

    def SetWheelHandler(self, callback):
        self._wheel_event_callback = callback

    def ClearWheelHandler(self):
        self._wheel_event_callback = None

