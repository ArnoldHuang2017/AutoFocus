# -- coding: utf-8 -
import Device.NucleusN as NucleusN
import Device.RealSense2 as RealSense2
from Core.Distance import *
from Core.ObjectDetector import *
from Core.FaceDetector import *
from Common import *
from Common import VAR
import time
import cv2
import numpy as np
from Device.RealSense2 import *

MODE_AUTOFOCUS = 1
MODE_MANUALFOCUS = 0

MODE_FOCUSCENTER = 1
MODE_FOCUS_AI = 2


class MainProcess(object):
    def __init__(self):
        self.STATUS = MODE_AUTOFOCUS
        self.LEN_CAL_ONGOING = False
        self.STOP_CAL_LEN = False
        self.CAPTURE_A_LEN_DATA = False
        self.calibration_points = Queue(10)
        self.last_motor_position = 9999
        self.yolo_mode = YoloFastV2()
        self.face_detector = FaceDetector()
        self.last_capture_time = 0
        self.focus_method = MODE_FOCUSCENTER

    def CALButtonLongPressHandler(self):
        if not self.LEN_CAL_ONGOING:
            print("Request CAL motor...")
            NucleusN.CalibrateMotor()
            time.sleep(10)
        else:
            self.LEN_CAL_ONGOING = False

    def CALButtonShortPressHandler(self):
        if self.LEN_CAL_ONGOING:
            self.CAPTURE_A_LEN_DATA = True
        else:
            if self.focus_method == MODE_FOCUSCENTER:
                print("Get into AI focus method.")
                self.focus_method = MODE_FOCUS_AI
            else:
                print("Get into center important area focus method.")
                self.focus_method = MODE_FOCUSCENTER

    def RecordOnButtonHandler(self):
        print("Switch to AutoFocus Mode...")
        self.STATUS = MODE_AUTOFOCUS

    def RecordOffButtonHandler(self):
        print("Switch to ManualFocus Mode...")
        self.STATUS = MODE_MANUALFOCUS

    def WheelHandler(self, data, position):
        if self.LEN_CAL_ONGOING or self.STATUS == MODE_MANUALFOCUS:
            self.last_motor_position = position
            NucleusN.WriteRawDataToMotor(data + "\n")

    def MainProcess(self, cal, lens_number, name, closest):
        # 原力N通信初始化，按键功能绑定
        print("Nucleus Init...")
        NucleusN.Start()
        NucleusN.SetCALButtonLongPressHandler(self.CALButtonLongPressHandler)
        NucleusN.SetCALButtonShortPressHandler(self.CALButtonShortPressHandler)
        NucleusN.SetRecordOnButtonLongPressHandler(self.RecordOnButtonHandler)
        NucleusN.SetRecordOffButtonLongPressHandler(self.RecordOffButtonHandler)
        NucleusN.SetWheelHandler(self.WheelHandler)
        time.sleep(5)
        # 电机初始化， 自动发送校准电机请求
        NucleusN.CalibrateMotor()
        time.sleep(12)

        lens_length = 58

        if lens_number is None:
            params = ReadYaml("/home/ubuntu/AutoFocus/len_param.yaml")
            if params:
                VAR.LENS_NUMBER = int(params["current_lens"])
                VAR.ROI_LENGTH = lens_length * 1.25
                lens_name = params[f"LENS_{VAR.LENS_NUMBER}"]["name"]
                VAR.LENS_LENGTH = int(params[f"LENS_{VAR.LENS_NUMBER}"]["length"])
                print(f"Will use the lens {VAR.LENS_NUMBER} {lens_name} ({lens_length}mm) to auto focus")
            else:
                lens_number = 1
        # 镜头初始化，目前没有镜头数据
        if cal:
            # 进入镜头标定流程
            self.LEN_CAL_ONGOING = True
            self.CreateNewLenData(lens_number, name, closest)
            exit()
        else:
            zoom_rate = VAR.ROI_LENGTH / 26
            VAR.CENTER_ROI_HEIGHT = int(RS2_HEIGHT / zoom_rate)
            VAR.CENTER_ROI_WIDTH = int(RS2_WIDTH / zoom_rate)
            VAR.CENTER_ROI_TOP_X = int((RS2_WIDTH - VAR.CENTER_ROI_WIDTH) / 2)
            VAR.CENTER_ROI_TOP_Y = int((RS2_HEIGHT - VAR.CENTER_ROI_HEIGHT) / 2)

            zoom_rate = VAR.LENS_LENGTH / 26.
            VAR.ROI_HEIGHT = int(RS2_HEIGHT / zoom_rate)
            VAR.ROI_WIDTH = int(RS2_WIDTH / zoom_rate)
            VAR.ROI_TOP_X = int((RS2_WIDTH - VAR.CENTER_ROI_WIDTH) / 2)
            VAR.ROI_TOP_Y = int((RS2_HEIGHT - VAR.CENTER_ROI_HEIGHT) / 2)

            t0 = cv2.getTickCount()
            print("Process the frames...")

            RealSense2.OpenCamera()

            while True:
                if self.STATUS == MODE_AUTOFOCUS:
                    self.ProcessFrames(t0)
                else:
                    time.sleep(1)
        # NucleusN.Stop()

    def CreateNewLenData(self, lens_number, name, closest):
        print("Now get into the len calibration process.")
        print("Please let a person stand at the blue point of the image window. And rotate the wheel")
        print("at the Nucleus N Handle until you see the person clear in the camera.")
        print("You should capture 5 distance data for better effect.")
        print("Short Press CAL button to capture a sample. ")
        print("If you complete the calibration process, please long press CAL button to generate the formula.")
        RealSense2.OpenCamera()
        lens_data = ReadYaml("/home/ubuntu/AutoFocus/len_param.yaml")
        print(lens_data)

        while True:
            color_frame, depth_frame = RealSense2.ReadCamera()
            center_distance = int(1000 * depth_frame.get_distance(int(RS2_WIDTH / 2), int(RS2_HEIGHT / 2)))
            # print(center_distance)
            color_image = np.asanyarray(color_frame.get_data())

            cv2.putText(color_image, "%d" % center_distance, (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (200, 200, 255), 3)
            cv2.circle(color_image, (int(RS2_WIDTH / 2), int(RS2_HEIGHT / 2)), 6, (255, 0, 0), -1)

            cal_size = self.calibration_points.Size()
            if cal_size == 0:
                cv2.putText(color_image, f"CAL at {closest}m", (int(RS2_WIDTH / 2), int(RS2_HEIGHT / 2) - 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            elif cal_size == 1:
                cv2.putText(color_image, "CAL at 1.5m", (int(RS2_WIDTH / 2), int(RS2_HEIGHT / 2) - 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            elif cal_size == 2:
                cv2.putText(color_image, "CAL at 2.0m", (int(RS2_WIDTH / 2), int(RS2_HEIGHT / 2) - 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            elif cal_size == 3:
                cv2.putText(color_image, "CAL at 2.5m", (int(RS2_WIDTH / 2), int(RS2_HEIGHT / 2) - 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            elif cal_size == 4:
                cv2.putText(color_image, "CAL at 3.0m", (int(RS2_WIDTH / 2), int(RS2_HEIGHT / 2) - 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            elif cal_size == 5:
                cv2.putText(color_image, "CAL at 3.5m", (int(RS2_WIDTH / 2), int(RS2_HEIGHT / 2) - 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            cv2.waitKey(1)
            last_position = self.last_motor_position
            if self.CAPTURE_A_LEN_DATA and last_position != 0:
                if not self.calibration_points.IsEmpty():
                    if self.calibration_points.Back().get(
                            "pos") - 5 < last_position < self.calibration_points.Back().get("pos") + 5:
                        print("Pulse delta is too small. ignore...")
                        print(f"last position data in cal data: {self.calibration_points.Back().get('pos')}")
                        print(f"current position {last_position}")

                    else:
                        data = {"dis": center_distance, "pos": last_position}
                        self.calibration_points.PushBack(data)
                        print(self.calibration_points.Back())
                else:
                    data = {"dis": center_distance, "pos": last_position}
                    self.calibration_points.PushBack(data)
                    print(self.calibration_points.Back())

                if cal_size == 6:
                    print("Capture enough data, quit the lens calibration.")
                    self.LEN_CAL_ONGOING = False
                else:
                    print(f"Get {cal_size} data.")
                self.CAPTURE_A_LEN_DATA = False

            if not self.LEN_CAL_ONGOING and cal_size == 6:
                self.CAPTURE_A_LEN_DATA = False
                data_list = []
                for i in range(0, 6):
                    item = self.calibration_points.GetIndex(i)
                    if item:
                        data_list.append(item["pos"])

                lens_key = f"LENS_{lens_number}"
                try:
                    lens_data[lens_key]["row"] = len(data_list)
                    lens_data[lens_key]["data"] = data_list
                    lens_data[lens_key]["name"] = name
                    lens_data[lens_key]["closest"] = closest * 1000
                    WriteYaml("/home/ubuntu/AutoFocus/len_param.yaml", lens_data)
                    print("The lens data is updated into the configuration file.")
                except:
                    print(lens_data)
                    raise

                break
            cv2.imshow("color", color_image)
        RealSense2.CloseCamera()

    def TimeTrigger(self, t0, fps):
        run_time = 1000 * (cv2.getTickCount() - t0) / cv2.getTickFrequency()

        delta_time = 1000 / fps
        if run_time - self.last_capture_time > delta_time:
            # print(f"Trigger time : {run_time}")
            self.last_capture_time = run_time
            return True
        return False
        # return False

    def CenterAreaFocus(self, color_image, depth_frame):

        # print(RIO)
        # (243, 126, 400.8727272727273, 226.9090909090909)
        # print(ROI_tl_x, ROI_tl_y, ROI_WIDTH, ROI_HEIGHT)

        stride = int(float(VAR.CENTER_ROI_WIDTH) // 25.)
        if stride == 0:
            stride = 1
        # cv2.circle(color_image, (int(RS2_WIDTH / 2), int(RS2_HEIGHT / 2)), 6, (255, 0, 0), -1)
        min_distance = 1000 * depth_frame.get_distance(int(RS2_WIDTH / 2), int(RS2_HEIGHT / 2))
        # print(f"Center distance: {min_distance}")
        center_distance = min_distance

        distances = []
        # cnt = 0

        # scan the area, the net grid x 10 on width direction
        for i in range(VAR.CENTER_ROI_TOP_X, VAR.CENTER_ROI_TOP_X + VAR.CENTER_ROI_WIDTH, stride):
            for j in range(VAR.CENTER_ROI_TOP_Y, VAR.CENTER_ROI_TOP_Y + VAR.CENTER_ROI_HEIGHT, stride):
                current_distance = 1000 * depth_frame.get_distance(i, j)
                if current_distance < min_distance and current_distance != 0:
                    min_distance = current_distance

        # print(f"Detected {cnt} in the center area")
        # cv2.circle(color_image, (center_x, center_y), 6, (255, 0, 0), -1)
        # print(f"center area distances: {min_distance}")

        return min_distance, center_distance

    def ProcessFrames(self, t0):
        fps = 24
        t1 = cv2.getTickCount()
        color_frame, depth_frame = RealSense2.ReadCamera()
        color_image = np.asanyarray(color_frame.get_data())

        cropped_image = color_image[VAR.ROI_TOP_Y:VAR.ROI_TOP_Y + VAR.ROI_HEIGHT,
                        VAR.ROI_TOP_X:VAR.ROI_TOP_X + + VAR.ROI_WIDTH]

        target_distance = 1000
        motor_position = 0

        if self.TimeTrigger(t0, fps):
            mode = ""
            is_print_fps = False
            if self.focus_method == MODE_FOCUSCENTER:
                mode = "MODE_FOCUSCENTER"
                target_distance, center_distance = self.CenterAreaFocus(color_image, depth_frame)
                target_distance = int(target_distance)
                # print(f"Center: {target_distance}mm")
            else:
                mode = "MODE_FOCUS_AI"
                target_distance, center_distance = self.CenterAreaFocus(color_image, depth_frame)

                if target_distance > 600:
                    cropped_image, face_distance_list = self.face_detector.detect(cropped_image, depth_frame)
                    if len(face_distance_list) > 0:
                        is_print_fps = True
                        face_distance_numbers = [item["face"] for item in face_distance_list]
                        eyes_distance_numbers = [num for elem in face_distance_list for num in elem["eyes"]]
                        if face_distance_numbers:
                            target_distance = int(min(face_distance_numbers))
                            print(f"================\n\n\n\n\nGet Face\n\n\n\n\n================")

                        if eyes_distance_numbers:
                            target_distance = int(min(eyes_distance_numbers))
                            print(f"================\n\n\n\n\nGet Eye\n\n\n\n\n================")

                    else:
                        outputs = self.yolo_mode.detect(cropped_image)
                        # cropped_image, obj_distance_list = self.yolo_mode.postprocess(cropped_image, outputs, depth_frame)
                        obj_distance_list = []
                        if len(obj_distance_list) > 0:
                            is_print_fps = True
                            target_distance = int(min(obj_distance_list))
                            print(f"================\n\n\n\n\nGet Object\n\n\n\n\n================")

            validated_distance = Distance().Calculate(target_distance)
            # print(f"Valid Distance: {target_distance}mm , After Kalman Filter: {validated_distance}")

            if validated_distance < 600:
                motor_position = int(self.Lagrange(600))

            elif validated_distance > 9500:
                motor_position = 9900
            else:
                motor_position = int(self.Lagrange(validated_distance))

            if motor_position < 0:
                motor_position = 400
            if motor_position > 9999:
                motor_position = 9900

                # timestamp = (cv2.getTickCount() - t0) * 1000 / cv2.getTickFrequency()
            print(f"Goto new position: from {self.last_motor_position} to  {motor_position} for {validated_distance}")
            self.last_motor_position = motor_position
            NucleusN.MoveMotor(motor_position)

            t2 = cv2.getTickCount()
            actually_fps = int(1000 / ((t2 - t1) * 1000 / cv2.getTickFrequency()))
            if is_print_fps:
                print(f"{t2 * 1000}: {mode} Actually FPS: {actually_fps}")
        #
        # cv2.imshow("cropped_image", cropped_image)
        # cv2.imshow("color_image", color_image)
        # cv2.waitKey(1)

    def Lagrange(self, distance, sample_count=7):
        y_result = 0.
        # 600mm is the minimum distance which lens supports
        data = ReadYaml("/home/ubuntu/AutoFocus/len_param.yaml")

        if not data:
            return 1000

        else:
            lens_params = data.get(f"LENS_{VAR.LENS_NUMBER}")
            if not lens_params:
                return 1000
            min_distance = float(lens_params["closest"])
            lens_distance_data = lens_params["data"]

        if distance >= min_distance:
            arr_x = [min_distance, 1500., 2000., 2500., 3000., 3500., 30000.]
            arr_y = [float(lens_distance_data[0]),
                     float(lens_distance_data[1]),
                     float(lens_distance_data[2]),
                     float(lens_distance_data[3]),
                     float(lens_distance_data[4]),
                     float(lens_distance_data[5]),
                     9800.,
                     ]

            LValue = [0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]

            for k in range(0, sample_count):
                temp1 = 1.0
                temp2 = 1.0
                for m in range(0, sample_count):
                    if m == k:
                        continue
                    temp1 *= (distance - arr_x[m])
                    temp2 *= arr_x[k] - arr_x[m]
                LValue[k] = temp1 / temp2

            for i in range(0, sample_count):
                y_result += arr_y[i] * LValue[i]

            return y_result
        else:
            return lens_distance_data[0]
