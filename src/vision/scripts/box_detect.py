#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from vision.msg import box_data
import torch
import cv2
from cv_bridge import CvBridge, CvBridgeError
import os

yolov5_path = str(os.getenv("YOLOV5_PATH"))

model = torch.hub.load(
    yolov5_path,
    "custom",
    path=f"{yolov5_path}/runs/train/box_model/weights/best.pt",
    source="local"
)
bridge = CvBridge()


def image_callback(data):
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)
        return

    results = model(cv_image)
    detections = results.xyxy[0].cpu().numpy()

    img_center_x = cv_image.shape[1] // 2
    img_center_y = cv_image.shape[0] // 2

    closest_detection = None
    min_distance = float("inf")

    for det in detections:
        x1, y1, x2, y2, conf, cls = det
        box_center_x = (x1 + x2) / 2
        box_center_y = (y1 + y2) / 2
        distance = (
            (box_center_x - img_center_x) ** 2 + (box_center_y - img_center_y) ** 2
        ) ** 0.5

        if distance < min_distance:
            min_distance = distance
            closest_detection = det

    box_msg = box_data()

    if closest_detection is not None:
        x1, y1, x2, y2, conf, cls = closest_detection
        box_center_x = (x1 + x2) / 2
        box_center_y = (y1 + y2) / 2
        box_msg.class_id = int(cls)
        box_msg.x = int(box_center_x - img_center_x)
        box_msg.y = int(box_center_y - img_center_y)
    else:
        box_msg.class_id = -1
        box_msg.x = 0
        box_msg.y = 0

    box_pub.publish(box_msg)

    for det in detections:
        x1, y1, x2, y2, conf, cls = det
        cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        cv2.putText(
            cv_image,
            f"{int(cls)}",
            (int(x1), int(y1) - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (36, 255, 12),
            2,
        )

    try:
        img_msg = bridge.cv2_to_imgmsg(cv_image, "bgr8")
        image_pub.publish(img_msg)
    except CvBridgeError as e:
        rospy.logerr(e)


rospy.init_node("box_detect", anonymous=True)
image_pub = rospy.Publisher("/yolov5/img", Image, queue_size=10)
box_pub = rospy.Publisher("/yolov5/box_detect", box_data, queue_size=10)
image_sub = rospy.Subscriber("/camera/ground", Image, image_callback)
rospy.spin()
