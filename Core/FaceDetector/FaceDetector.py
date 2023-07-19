# -- coding: utf-8 -
import cv2
from Common import *
from Device.RealSense2 import RS2_HEIGHT, RS2_WIDTH, RS2_HEIGHT_COMPENSATE, RS2_WIDTH_COMPENSATE


@singleton
class FaceDetector(object):
    def __init__(self, scale_factor=1.1, min_neighbors=5):
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_alt2.xml")
        self.profile_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_profileface.xml")
        self.eye_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_eye.xml")
        self.scale_factor = scale_factor
        self.min_neighbors = min_neighbors

        self.nod_count = 0

    def calculate_intersection(self, box1, box2):
        x1 = max(box1[0], box2[0])
        y1 = max(box1[1], box2[1])
        x2 = min(box1[0] + box1[2], box2[0] + box2[2])
        y2 = min(box1[1] + box1[3], box2[1] + box2[3])

        intersection_width = max(0, x2 - x1)
        intersection_height = max(0, y2 - y1)

        intersection_area = intersection_width * intersection_height

        return intersection_area

    def calculate_union(self, box1, box2):
        # 计算两个物体框的并集面积
        area_box1 = box1[2] * box1[3]
        area_box2 = box2[2] * box2[3]

        union_area = area_box1 + area_box2 - self.calculate_intersection(box1, box2)

        return union_area

    def calculate_iou(self, box1, box2):
        intersection_area = self.calculate_intersection(box1, box2)
        union_area = self.calculate_union(box1, box2)

        iou = intersection_area / union_area

        return iou

    def remove_duplicate_boxes(self, boxes, threshold):
        non_duplicate_boxes = []

        for i in range(len(boxes)):
            duplicate = False
            box1 = boxes[i]

            for j in range(i + 1, len(boxes)):
                box2 = boxes[j]
                iou = self.calculate_iou(box1, box2)

                if iou > threshold:
                    duplicate = True
                    break

            if not duplicate:
                non_duplicate_boxes.append(box1)

        return non_duplicate_boxes

    def remove_nod_box(self, boxes):
        sorted_boxes = sorted(boxes, key=lambda box: box[1] + box[3] / 2)
        sorted_boxes = sorted_boxes[:-1]
        return sorted_boxes

    def detect_eye(self, color_image, gray, face):

        (x, y, w, h) = face
        roi_gray = gray[y:y + h, x:x + w]
        eyes = self.eye_cascade.detectMultiScale(roi_gray)
        eyes = self.remove_duplicate_boxes(eyes, 0.3)
        if len(eyes) > 2:
            eyes = self.remove_nod_box(eyes)

        final_eyes = []
        for (ex, ey, ew, eh) in eyes:
            aspect_ratio = ew / eh
            if 0.5 <= aspect_ratio <= 1.5:
                final_eyes.append((ex, ey, ew, eh))
        return eyes

    def is_point_inside_box(self, box_x, box_y, box_width, box_height, point_x, point_y):
        if box_x <= point_x <= (box_x + box_width) and box_y <= point_y <= (box_y + box_height):
            return True
        else:
            return False

    def detect(self, color_image, depth_frame):
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, scaleFactor=self.scale_factor, minNeighbors=self.min_neighbors)

        all_faces_data = []

        for face in faces:
            # Personal decide that won't to detect too small face.
            distance_data = {"face": None, "eyes": []}
            (x, y, w, h) = face
            if w <= 100 or h <= 100:
                continue

            center_face_x = x + w // 2
            center_face_y = y + h // 2

            if not self.is_point_inside_box(x, y, w, h, center_face_x, center_face_y):
                continue

            distance_data["face"] = depth_frame.get_distance(center_face_x + VAR.ROI_TOP_X,
                                                             center_face_y + VAR.ROI_TOP_Y) * 1000

            # 此处的x y w h 是脸在裁切后的图像 color里的坐标， 所以方框直接画就行。
            cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 0, 255), 1)

            # detect eyes on face only
            eyes = self.detect_eye(color_image, gray, face)

            for (ex, ey, ew, eh) in eyes:
                # 此处的ex ey ew eh 是在脸的截图中的坐标，所以需要还原到脸被截图之前的坐标系
                center_eye_x = ex + ew // 2
                center_eye_y = ey + eh // 2

                distance_data["eyes"].append(
                    depth_frame.get_distance(x + center_eye_x + VAR.ROI_TOP_X, y + center_eye_y + VAR.ROI_TOP_Y) * 1000)

                cv2.rectangle(color_image, (x + ex, y + ey), (x + ex + ew, y + ey + eh), (0, 0, 255), 1)
                cv2.circle(color_image, (x + center_eye_x, y + center_eye_y), 6, (255, 0, 0), -1)

            if len(eyes) > 2:
                # if detect more than 2 eyes after remove possible nod
                pass

            all_faces_data.append(distance_data)

        if len(faces) == 0:
            profiles = self.profile_cascade.detectMultiScale(gray, scaleFactor=self.scale_factor,
                                                             minNeighbors=self.min_neighbors)
            for face in profiles:
                # Personal decide that won't to detect too small face.
                distance_data = {"face": None, "eyes": []}
                (x, y, w, h) = face

                if w <= 100 or h <= 100:
                    continue

                cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 0, 255), 1)

                center_face_x = x + w // 2
                center_face_y = y + h // 2
                distance_data["face"] = depth_frame.get_distance(center_face_x + VAR.ROI_TOP_X,
                                                                 center_face_y + VAR.ROI_TOP_Y) * 1000

                cv2.circle(color_image, (center_face_x, center_face_y), 6, (255, 0, 0),
                           -1)

                # detect eyes on face only
                eyes = self.detect_eye(color_image, gray, face)

                for (ex, ey, ew, eh) in eyes:
                    center_eye_x = ex + ew // 2
                    center_eye_y = ey + eh // 2
                    distance_data["eyes"].append(depth_frame.get_distance(x + center_eye_x + VAR.ROI_TOP_X,
                                                                          y + center_eye_y + VAR.ROI_TOP_Y) * 1000)
                    cv2.rectangle(color_image, (x + ex, y + ey), (x + ex + ew, y + ey + eh), (0, 0, 255), 1)
                    cv2.circle(color_image, (x + center_eye_x, y + center_eye_y), 6, (255, 0, 0), -1)

                if len(eyes) > 2:
                    # if detect more than 2 eyes after remove possible nod
                    pass
        # cv2.waitKey(1)
        # cv2.imshow("face", color_image)
        return color_image, all_faces_data
