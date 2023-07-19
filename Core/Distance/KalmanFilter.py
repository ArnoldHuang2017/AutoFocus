import numpy as np
from Common import *


@singleton
class KalmanFilter(object):
    def __init__(self, process_error=1e-2, measure_error=1e-2):
        self.A = np.eye(2)
        self.AT = np.eye(2)
        self.Q = np.eye(2) * process_error
        self.R = np.eye(2) * measure_error
        self.P = np.eye(2) * 1e7

    def init(self, state, error):
        self.xhat = state
        self.xhatminus = self.xhat
        self.P = np.eye(2) * error
        self.Pminus = self.P

    def setA(self, _A):
        self.A = _A
        self.AT = self.A.T

    def setH(self, _H):
        self.H = _H
        self.HT = self.H.T

    def predict(self):
        # Ԥ��״̬
        self.xhatminus = np.dot(self.A, self.xhat)

        # Ԥ�����Э����
        self.Pminus = np.dot(np.dot(self.A, self.P), self.AT) + self.Q

        return self.xhatminus

    def update(self, measurement):
        self.z = measurement

        # ���㿨��������
        K = np.dot(np.dot(self.Pminus, self.HT), np.linalg.inv(np.dot(np.dot(self.H, self.Pminus), self.HT) + self.R))

        # �ò���ֵ���¹���
        self.xhat = self.xhatminus + np.dot(K, self.z - np.dot(self.H, self.xhatminus))

        # �������Э����
        self.P = np.dot((self.Q - np.dot(K, self.H)), self.Pminus)

        return self.xhat
