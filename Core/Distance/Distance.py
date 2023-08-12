from Common import *
from .KalmanFilter import KalmanFilter
import cv2
import numpy as np

DISTANCE_QUEUE_LENGTH = 150
ERROR_QUEUE_LENGTH = 10
TIMESTAMP_QUEUE_LENGTH = 10
KALMAN_FILTER_ON = True


@singleton
class Distance(object):
    def __init__(self):
        self.target_distance = Queue(size=DISTANCE_QUEUE_LENGTH + 1)
        self.error_distance = Queue(size=ERROR_QUEUE_LENGTH)
        self.timestamp = Queue(TIMESTAMP_QUEUE_LENGTH)
        self.distance_filter = KalmanFilter(0.2, 0.1)
        self.last_distance = 1000
        self.filter_init = False
        self.filter_mean = None
        self.filter_cov = None
        self.distance_filter.setH(np.eye(2, dtype=np.float32))

    def Calculate(self, rs2_distance):
        # print("target_distance List:")
        # self.target_distance.Show()
        # print("error_distance List:")
        # self.error_distance.Show()

        if self.target_distance.Size() >= DISTANCE_QUEUE_LENGTH:
            # print("distance queue is full, delete the oldest")
            self.target_distance.PopFront()

        if not KALMAN_FILTER_ON:
            if self.IsValid(rs2_distance):
                self.target_distance.PushBack(rs2_distance)
                return rs2_distance
            else:
                return self.target_distance.Back()
        else:
            self.timestamp.PushFront(cv2.getTickCount())

            if self.timestamp.Size() > TIMESTAMP_QUEUE_LENGTH:
                self.timestamp.PopBack()

            if rs2_distance != 0:
                if self.IsValid(rs2_distance):
                    # print("distance is correct")
                    self.target_distance.PushBack(rs2_distance)
                    self.UpdateFilter()
                    return self.target_distance.Back()
                elif not self.target_distance.IsEmpty():
                    # print("distance is not valid")
                    self.target_distance.PushBack(self.target_distance.Back())
                    self.UpdateFilter()
                    return self.target_distance.Back()
                # print("distance is not valid")

            elif not self.target_distance.IsEmpty():
                self.target_distance.PushBack(self.target_distance.Back())
                self.UpdateFilter()
                return self.target_distance.Back()
            else:
                self.target_distance.PushBack(1000)
                # print("Out of detect limit")
                return -1
        return 0

    def IsValid(self, current_distance):
        if self.error_distance.Size() > 2:
            # print("Detect two error distance, get into a new platform.")
            self.error_distance.Clear()
            self.target_distance.Clear()
            self.target_distance.PushBack(current_distance)
            print("\n\n\n====clear====\n\n\n")
            return True

        if not self.error_distance.IsEmpty():
            # if the current distance delta is lower than 10% * last error distance, then add it into error queue
            if abs(current_distance - self.error_distance.Back()) < self.error_distance.Back() * 0.1:
                # print(
                #    f"Current distance delta {abs(current_distance - self.error_distance.Back())} is less than 10% of last error distance {self.error_distance.Back()}. Error!")
                self.error_distance.PushBack(current_distance)
                if self.error_distance.Size() <= 2:
                    # print("Return False")
                    return False

        if self.target_distance.Size() > 1:
            delta = abs(self.target_distance.GetIndex(
                self.target_distance.Size() - 1) - current_distance)
            if delta > self.target_distance.GetIndex(
                    self.target_distance.Size() - 1) * 0.25:
                # print(
                #    f"Current distance delta {delta} is larger than 25% of last distance {self.target_distance.GetIndex(self.target_distance.Size() - 1)}. Error!")
                self.error_distance.PushBack(current_distance)
                # print("Return False")
                return False

        return True

    def UpdateFilter(self):
        distance = self.target_distance.Back()

        if not self.filter_init:
            init_vec = np.array([[float(distance)], [0]], dtype=np.float32)
            self.distance_filter.init(init_vec, 1e-2)
            self.filter_init = True
        else:
            delta_time = float((self.timestamp.GetIndex(0) - self.timestamp.GetIndex(1)) / cv2.getTickFrequency())
            self.distance_filter.setA(np.array([[1, delta_time], [0, 1]], dtype=np.float32))
            delta_distance = self.last_distance - distance
            self.distance_filter.predict()
            correct_vec = self.distance_filter.update(
                np.array([[float(distance)], [float(delta_distance)]], dtype=np.float32))

            if 9500 >= correct_vec[0][0] >= 500:
                self.target_distance.UpdateBack(correct_vec[0][0])
                distance = correct_vec[0][0]

        self.last_distance = distance
