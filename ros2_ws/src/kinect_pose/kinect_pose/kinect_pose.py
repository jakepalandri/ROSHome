from enum import IntEnum
from ultralytics import YOLO
import torch
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from message_filters import TimeSynchronizer, Subscriber
import numpy as np
from cv_bridge import CvBridge
bridge = CvBridge()

class Point(IntEnum):
    NOSE:           int = 0
    LEFT_EYE:       int = 1
    RIGHT_EYE:      int = 2
    LEFT_EAR:       int = 3
    RIGHT_EAR:      int = 4
    LEFT_SHOULDER:  int = 5
    RIGHT_SHOULDER: int = 6
    LEFT_ELBOW:     int = 7
    RIGHT_ELBOW:    int = 8
    LEFT_WRIST:     int = 9
    RIGHT_WRIST:    int = 10
    LEFT_HIP:       int = 11
    RIGHT_HIP:      int = 12
    LEFT_KNEE:      int = 13
    RIGHT_KNEE:     int = 14
    LEFT_ANKLE:     int = 15
    RIGHT_ANKLE:    int = 16

class ReadKinectPose(Node):
    def __init__(self):
        super().__init__('read_kinect_pose')
        self.model = YOLO("yolov8m-pose.pt")
        self.model = self.model.to("cuda" if torch.cuda.is_available() else "cpu")
        colour_image = Subscriber(self, Image, "/kinect2/sd/image_color_rect")
        depth_image = Subscriber(self, Image, "/kinect2/sd/image_depth")
        self.tss = TimeSynchronizer([colour_image, depth_image], 10)
        self.tss.registerCallback(self.listener_callback)

    def listener_callback(self, image, depth):
        stream = bridge.imgmsg_to_cv2(image, "bgr8")
        results = self.model(source=stream, show=True)
        isPerson = results[0].keypoints.has_visible
        result_keypoint = results[0].keypoints.xyn.cpu().numpy()[0]
        if (not isPerson):
            print("No person detected")
            return
        # LOGIC:
        # 1. Get the coordinates of the person's right wrist and right elbow
        right_wrist = result_keypoint[Point.RIGHT_WRIST]
        right_elbow = result_keypoint[Point.RIGHT_ELBOW]
        print("Right wrist: ", right_wrist)
        print("Right elbow: ", right_elbow)
        # 2. Get the coordinates of the person's left wrist and left elbow
        left_wrist = result_keypoint[Point.LEFT_WRIST]
        left_elbow = result_keypoint[Point.LEFT_ELBOW]
        print("Left wrist: ", left_wrist)
        print("Left elbow: ", left_elbow)
        # 3. Calculate the vector from the right elbow to the right wrist
        right_vector = right_wrist - right_elbow
        # 4. Calculate the vector from the left elbow to the left wrist
        left_vector = left_wrist - left_elbow
        # 5. Determine which arm is more extended
        # 5a. Determine distance from right wrist to right hip
        right_hip = result_keypoint[Point.RIGHT_HIP]
        right_distance = np.linalg.norm(right_wrist - right_hip)
        # 5b. Determine distance from left wrist to left hip
        left_hip = result_keypoint[Point.LEFT_HIP]
        left_distance = np.linalg.norm(left_wrist - left_hip)
        # 5c. Compare the distances
        # NEED A MORE ADVANCED WAY TO DO THIS
        # NEED A MINIMUM EXTENSION THRESHOLD
        # Default to right arm
        vector = right_vector
        if left_distance > right_distance:
            vector = left_vector
            print("Left arm is more extended")
            print(vector)
        else:
            print("Right arm is more extended")
            print(vector)

        # 6. Get depth data at the right wrist and right elbow with some margin of error (maybe check the closest depth value (not 0) within a range)
        # 7. Get depth data at the left wrist and left elbow
        # 8. Calculate vector from right elbow to right wrist in 3D space
        # 9. Calculate vector from left elbow to left wrist in 3D space
        # 10. Extend the vector to the ends of the room
        # 11. Determine what the person is pointing at with some margin of error


def main(args=None):
    rclpy.init(args=args)

    read_kinect_pose = ReadKinectPose()

    rclpy.spin(read_kinect_pose)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    read_kinect_pose.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()