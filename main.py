import Core.Process as Process
import time
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="arguments")
    parser.add_argument("--cal", help="Please input lens index number")
    parser.add_argument("--name", help="Please input lens name")
    parser.add_argument("--min", help="Please input the closest focus distance by meter.")

    args = parser.parse_args()

    if args.cal and args.name and args.min:
        Process.Start(True, args.cal, args.name, float(args.min))
    else:
        Process.Start(False)


# import cv2
# import numpy as np
# from Core.FaceDetectLight import FaceRecognition
# from Device import RealSense2
#
# if __name__ == '__main__':
#
#     dete_model = r'/home/ubuntu/AutoFocus/Data/scrfd_500m_kps_320.onnx'
#     reco_model = r'/home/ubuntu/AutoFocus/Data/scrfd_500m_kps.onnx'
#
#     face_reco = FaceRecognition(detector_model=dete_model, recognition_model=reco_model, ctx_id=0)
#     RealSense2.OpenCamera()
#     while True:
#         color_frame, depth_frame = RealSense2.ReadCamera()
#         color_image = np.asanyarray(color_frame.get_data())
#         results = face_reco.detect(color_image, depth_frame)
#
#         for result in results:
#             x1, y1, x2, y2 = result["bbox"]
#             cv2.rectangle(color_image, (x1, y1), (x2, y2), (255, 0, 0), 2)
#             cv2.putText(img=color_image, text="face", org=(x1, y1 - 10),
#                         fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(0, 255, 0), thickness=1)
#             for landmark in result["landmark"]:
#                 cv2.circle(color_image, tuple(landmark), 1, (0, 0, 255), 2)
#
#         cv2.imshow('Image', color_image)
#         cv2.waitKey(1)
#
#     # cv2.destroyAllWindows()
