# -- coding: utf-8 -
import cv2
import numpy as np
from Common import *
from Device.RealSense2 import RS2_HEIGHT, RS2_WIDTH, RS2_HEIGHT_COMPENSATE, RS2_WIDTH_COMPENSATE


class YoloFastV2(object):
    def __init__(self, objThreshold=0.45, confThreshold=0.45, nmsThreshold=0.4):
        with open('/home/ubuntu/AutoFocus/Data/coco.names', 'rt') as f:
            self.classes = f.read().rstrip('\n').split('\n')
        self.stride = [16, 32]
        self.anchor_num = 3
        self.anchors = np.array(
            [12.64, 19.39, 37.88, 51.48, 55.71, 138.31, 126.91, 78.23, 131.57, 214.55, 279.92, 258.87],
            dtype=np.float32).reshape(len(self.stride), self.anchor_num, 2)
        self.inpWidth = 352
        self.inpHeight = 352
        self.net = cv2.dnn.readNet('/home/ubuntu/AutoFocus/Data/model.onnx')
        self.confThreshold = confThreshold
        self.nmsThreshold = nmsThreshold
        self.objThreshold = objThreshold

    def _make_grid(self, nx=20, ny=20):
        xv, yv = np.meshgrid(np.arange(ny), np.arange(nx))
        return np.stack((xv, yv), 2).reshape((-1, 2)).astype(np.float32)

    def is_point_inside_box(self, box_x, box_y, box_width, box_height, point_x, point_y):
        if box_x <= point_x <= (box_x + box_width) and box_y <= point_y <= (box_y + box_height):
            return True
        else:
            return False

    def area(self, width, height):
        return width * height

    def postprocess(self, frame, outs, depth):
        frameHeight = frame.shape[0]
        frameWidth = frame.shape[1]
        ratioh, ratiow = frameHeight / self.inpHeight, frameWidth / self.inpWidth
        # Scan through all the bounding boxes output from the network and keep only the
        # ones with high confidence scores. Assign the box's class label as the class with the highest score.
        classIds = []
        confidences = []
        boxes = []

        for detection in outs:
            scores = detection[5:]
            classId = np.argmax(scores)
            confidence = scores[classId]
            if confidence > self.confThreshold and detection[4] > self.objThreshold:
                center_x = int(detection[0] * ratiow)
                center_y = int(detection[1] * ratioh)
                width = int(detection[2] * ratiow)
                height = int(detection[3] * ratioh)
                left = int(center_x - width / 2)
                top = int(center_y - height / 2)
                classIds.append(classId)

                # 875 = 25 * 35, MIN AREA
                if self.area(width, height) > 875:
                    # print(f"Detect object! Area: {self.area(width, height) }")
                    confidences.append(float(confidence * detection[4]))
                    boxes.append([left, top, width, height])
                    # cv2.rectangle(frame, (left, top), (left + width, top + height), (0, 0, 255), 1)
                    # cv2.circle(frame, (center_x, center_y), 6, (0, 0, 255), -1)

        # Perform non maximum suppression to eliminate redundant overlapping boxes with
        # lower confidences.
        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.confThreshold, self.nmsThreshold)

        object_distances = []
        object_types = []

        for i in indices:
            box = boxes[i]
            left = int(box[0])
            top = int(box[1])
            width = int(box[2])
            height = int(box[3])
            class_id = classIds[i]
            print(f"Object class:{class_id}")

            min_length = min(width, height)
            wh_ratio = float(width) / float(height)

            if min_length <= 20:
                center_x = left + width // 2 + VAR.ROI_TOP_X
                center_y = top + height // 2 + VAR.ROI_TOP_Y
                object_distances.append(depth.get_distance(center_x, center_y) * 1000)
                continue

            if class_id == 0:
                # if it's a person
                print("Detect person body!")
                k = 0.5
                height = int(k * wh_ratio * height)

            stride = int(min(height, width) / 10)

            body_distances = []
            position = []

            for i in range(left, left + width, stride):
                for j in range(top, top + height, stride):
                    current_distance = 1000 * depth.get_distance(i + VAR.ROI_TOP_X, j + VAR.ROI_TOP_Y)
                    if current_distance != 0 and current_distance < 25000:
                        body_distances.append(current_distance)
                        position.append((i, j))

            body_distances.sort()

            location = int(len(body_distances) / 2)

            if class_id == 0:
                location = int(len(body_distances) / 6)

            print(f"Take the location {location} : {body_distances[location]} as min distance.")
            # cv2.circle(frame, position[location], 6, (255, 0, 0), -1)
            # cv2.waitKey(1)
            # cv2.imshow("yolo", frame)

            object_distances.append(body_distances[location])

        return frame, object_distances

    def detect(self, srcimg):
        blob = cv2.dnn.blobFromImage(srcimg, 1 / 255.0, (self.inpWidth, self.inpHeight))
        self.net.setInput(blob)
        outs = self.net.forward(self.net.getUnconnectedOutLayersNames())[0]

        outputs = np.zeros((outs.shape[0] * self.anchor_num, 5 + len(self.classes)))
        row_ind = 0
        for i in range(len(self.stride)):
            h, w = int(self.inpHeight / self.stride[i]), int(self.inpWidth / self.stride[i])
            length = int(h * w)
            grid = self._make_grid(w, h)
            for j in range(self.anchor_num):
                top = row_ind + j * length
                left = 4 * j
                outputs[top:top + length, 0:2] = (outs[row_ind:row_ind + length,
                                                  left:left + 2] * 2. - 0.5 + grid) * int(self.stride[i])
                outputs[top:top + length, 2:4] = (outs[row_ind:row_ind + length,
                                                  left + 2:left + 4] * 2) ** 2 * np.repeat(
                    self.anchors[i, j, :].reshape(1, -1), h * w, axis=0)
                outputs[top:top + length, 4] = outs[row_ind:row_ind + length, 4 * self.anchor_num + j]
                outputs[top:top + length, 5:] = outs[row_ind:row_ind + length, 5 * self.anchor_num:]
            row_ind += length
        return outputs
