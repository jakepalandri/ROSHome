from enum import IntEnum
from ultralytics import YOLO
import torch
import math
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
        depth = bridge.imgmsg_to_cv2(depth, "passthrough")
        results = self.model(source=stream, show=True)
        isPerson = results[0].keypoints.has_visible
        result_keypoint = results[0].keypoints.xy.cpu().numpy()[0]
        if (not isPerson):
            print("No person detected")
            return
        # LOGIC:
        # 1. Get the coordinates of the person's right wrist, elbow and hip
        right_wrist = result_keypoint[Point.RIGHT_WRIST]
        right_elbow = result_keypoint[Point.RIGHT_ELBOW]
        right_hip = result_keypoint[Point.RIGHT_HIP]
        # 2. Get the coordinates of the person's left wrist, elbow and hip
        left_wrist = result_keypoint[Point.LEFT_WRIST]
        left_elbow = result_keypoint[Point.LEFT_ELBOW]
        left_hip = result_keypoint[Point.LEFT_HIP]
        # 3. Get depth data at the left and right wrist elbow and hip with some margin of error (maybe check the closest depth value (not 0) within a range)
        diameter = 5
        min_depth_right_elbow = math.inf
        min_depth_right_wrist = math.inf
        min_depth_right_hip = math.inf
        min_depth_left_elbow = math.inf
        min_depth_left_wrist = math.inf
        min_depth_left_hip = math.inf
        for i in range(-math.floor(diameter/2), math.ceil(diameter/2)):
            for j in range(-math.floor(diameter/2), math.ceil(diameter/2)):
                rounded_right_wrist = [ round(elem) for elem in right_wrist ]
                rounded_right_elbow = [ round(elem) for elem in right_elbow ]
                rounded_right_hip = [ round(elem) for elem in right_hip ]
                rounded_left_wrist = [ round(elem) for elem in left_wrist ]
                rounded_left_elbow = [ round(elem) for elem in left_elbow ]
                rounded_left_hip = [ round(elem) for elem in left_hip ]
                depth_right_wrist = depth[rounded_right_wrist[1] + i][rounded_right_wrist[0] + j]
                depth_right_elbow = depth[rounded_right_elbow[1] + i][rounded_right_elbow[0] + j]
                depth_right_hip = depth[rounded_right_hip[1] + i][rounded_right_hip[0] + j]
                depth_left_wrist = depth[rounded_left_wrist[1] + i][rounded_left_wrist[0] + j]
                depth_left_elbow = depth[rounded_left_elbow[1] + i][rounded_left_elbow[0] + j]
                depth_left_hip = depth[rounded_left_hip[1] + i][rounded_left_hip[0] + j]
                if (depth_right_wrist < min_depth_right_wrist and depth_right_wrist != 0):
                    min_depth_right_wrist = depth_right_wrist
                if (depth_left_hip < min_depth_left_hip and depth_left_hip != 0):
                    min_depth_left_hip = depth_left_hip
                if (depth_right_hip < min_depth_right_hip and depth_right_hip != 0):
                    min_depth_right_hip = depth_right_hip
                if (depth_left_elbow < min_depth_left_elbow and depth_left_elbow != 0):
                    min_depth_left_elbow = depth_left_elbow
                if (depth_left_wrist < min_depth_left_wrist and depth_left_wrist != 0):
                    min_depth_left_wrist = depth_left_wrist
                if (depth_right_elbow < min_depth_right_elbow and depth_right_elbow != 0):
                    min_depth_right_elbow = depth_right_elbow
        # 4. Convert the 2d coordinates to 3d coordinates with Z up and Y depth
        right_wrist = np.array([right_wrist[0], min_depth_right_wrist, right_wrist[1]])
        right_elbow = np.array([right_elbow[0], min_depth_right_elbow, right_elbow[1]])
        right_hip = np.array([right_hip[0], min_depth_right_hip, right_hip[1]])
        left_wrist = np.array([left_wrist[0], min_depth_left_wrist, left_wrist[1]])
        left_elbow = np.array([left_elbow[0], min_depth_left_elbow, left_elbow[1]])
        left_hip = np.array([left_hip[0], min_depth_left_hip, left_hip[1]])
        # 5. Calculate the 3d vector for the left and right forearms
        right_vector = right_wrist - right_elbow
        left_vector = left_wrist - left_elbow
        
        # 7. Determine which arm is more extended
        # 7a. Determine distance from right wrist to right hip
        right_distance = np.linalg.norm(right_wrist - right_hip)
        # 7b. Determine distance from left wrist to left hip
        left_distance = np.linalg.norm(left_wrist - left_hip)
        # 7c. Compare the distances
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