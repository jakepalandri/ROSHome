from enum import IntEnum
from ultralytics import YOLO
from .MqttClient import MqttClient
import torch
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
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
        self.min = [-2500, -1000, 0000]
        self.max = [ 2500,  3000, 5000]

        self.model = YOLO("yolov8m-pose.pt")
        self.model = self.model.to("cuda" if torch.cuda.is_available() else "cpu")

        # using HD camera, needs calibrating
        color_image = Subscriber(self, Image, "/kinect2/hd/image_color_rect")
        depth_image = Subscriber(self, Image, "/kinect2/hd/image_depth_rect")
        camera_info = Subscriber(self, CameraInfo, "/kinect2/hd/camera_info")

        self.tss = TimeSynchronizer([color_image, depth_image, camera_info], 10)
        self.tss.registerCallback(self.listener_callback)

        self.client = MqttClient()
        self.frames_since_last_pub = 0
        self.frames_holding_gesture = 0
        self.last_gesture = ""

    def listener_callback(self, image, depth, info):
        self.frames_since_last_pub += 1
        if self.frames_since_last_pub < 30:
            return

        stream = bridge.imgmsg_to_cv2(image, "bgr8")
        depth  = bridge.imgmsg_to_cv2(depth, "passthrough")
        rect_matrix = np.array(info.k).reshape(3, 3)
        results = self.model(source=stream, show=True)
        isPerson = results[0].keypoints.has_visible
        result_keypoint = results[0].keypoints.xy.cpu().numpy()[0]

        if not isPerson:
            print("No person detected")
            return
        
        # LOGIC:
        # 1. Get the coordinates of the person's right wrist, elbow, hip and shoulder
        right_wrist_pixel = result_keypoint[Point.RIGHT_WRIST   ]
        right_elbow_pixel = result_keypoint[Point.RIGHT_ELBOW   ]
        right_hip_pixel   = result_keypoint[Point.RIGHT_HIP     ]
        right_shldr_pixel = result_keypoint[Point.RIGHT_SHOULDER]

        # 2. Get the coordinates of the person's left wrist, elbow, hip and shoulder
        left_wrist_pixel  = result_keypoint[Point.LEFT_WRIST    ]
        left_elbow_pixel  = result_keypoint[Point.LEFT_ELBOW    ]
        left_hip_pixel    = result_keypoint[Point.LEFT_HIP      ]
        left_shldr_pixel  = result_keypoint[Point.LEFT_SHOULDER ]

        # 3. Get depth data at the left and right wrist elbow and hip with some margin of error
        right_wrist_z = self.min_depth(depth, right_wrist_pixel)
        right_elbow_z = self.min_depth(depth, right_elbow_pixel)
        right_hip_z   = self.min_depth(depth, right_hip_pixel  )
        right_shldr_z = self.min_depth(depth, right_shldr_pixel)
        left_wrist_z  = self.min_depth(depth, left_wrist_pixel )
        left_elbow_z  = self.min_depth(depth, left_elbow_pixel )
        left_hip_z    = self.min_depth(depth, left_hip_pixel   )
        left_shldr_z  = self.min_depth(depth, left_shldr_pixel )

        # 4. Convert 2d pixel coordinates to 3d world coordinates
        right_wrist_3d = self.pixel_to_world(right_wrist_pixel, rect_matrix, right_wrist_z)
        right_elbow_3d = self.pixel_to_world(right_elbow_pixel, rect_matrix, right_elbow_z)
        right_hip_3d   = self.pixel_to_world(right_hip_pixel  , rect_matrix, right_hip_z  )
        right_shldr_3d = self.pixel_to_world(right_shldr_pixel, rect_matrix, right_shldr_z)
        left_wrist_3d  = self.pixel_to_world(left_wrist_pixel , rect_matrix, left_wrist_z )
        left_elbow_3d  = self.pixel_to_world(left_elbow_pixel , rect_matrix, left_elbow_z )
        left_hip_3d    = self.pixel_to_world(left_hip_pixel   , rect_matrix, left_hip_z   )
        left_shldr_3d  = self.pixel_to_world(left_shldr_pixel , rect_matrix, left_shldr_z )

        # 5. Calculate the 3d vector for the left and right forearms
        right_vector = right_wrist_3d - right_elbow_3d
        left_vector  = left_wrist_3d  - left_elbow_3d
        
        # 6. Determine which arm is more extended
        # 6a. Determine distance from right wrist to hip and shoulder
        right_hip_distance   = np.linalg.norm(right_wrist_3d - right_hip_3d  )
        right_shldr_distance = np.linalg.norm(right_wrist_3d - right_shldr_3d)

        # 6b. Determine distance from left wrist to hip and shoulder
        left_hip_distance    = np.linalg.norm(left_wrist_3d  - left_hip_3d   )
        left_shldr_distance  = np.linalg.norm(left_wrist_3d  - left_shldr_3d )

        # 6c. Set extension distance thresholds
        hip_threshold   = 500
        shldr_threshold = 400

        # NEED A MORE ADVANCED WAY TO DO THIS
        # NEED A MINIMUM EXTENSION THRESHOLD -  CURRENTLY 500mm
        # DOESN'T ACCOUNT FOR IF THE PERSON'S SHOULDERS AND HIPS AREN'T VISIBLE
        # OR IF ONE SHOULDER AND OPPOSITE HIP ARE VISIBLE (DIAGONAL) - THIS SEEMS LIKE A RARE EDGE CASE
        # print("LWrist:    ", left_wrist_3d       )
        # print("LHip:      ", left_hip_3d         )
        # print("LSldr:     ", left_shldr_3d       )
        # print("RWrist:    ", right_wrist_3d      )
        # print("RHip:      ", right_hip_3d        )
        # print("RSldr:     ", right_shldr_3d      )
        # print("LHipDist:  ", left_hip_distance   )
        # print("RHipDist:  ", right_hip_distance  )
        # print("LSldrDist: ", left_shldr_distance )
        # print("RSldrDist: ", right_shldr_distance)
        # print("LVector:   ", left_vector         )
        # print("RVector:   ", right_vector        )

        # If hips aren't visible, use distance from shoulders
        left_distance  = left_hip_distance
        right_distance = right_hip_distance
        threshold = hip_threshold
        if np.isnan(left_distance) or np.isnan(right_distance):
            left_distance  = left_shldr_distance
            right_distance = right_shldr_distance
            threshold = shldr_threshold
            # print("Using shoulder dist")
        # else:
            # print("Using hip dist")
        
        # 6d. Compare the distances
        if (self.arm_is_extended(left_distance, threshold) and
            left_distance > right_distance and
            self.vector_is_valid(left_vector)):
            vector = left_vector
            origin = left_wrist_3d
            # print("Left arm is more extended")
        elif (self.arm_is_extended(right_distance, threshold) and
              right_distance >= left_distance and
              self.vector_is_valid(right_vector)):
            vector = right_vector
            origin = right_wrist_3d
            # print("Right arm is more extended")
        else:
            # print("Person is pointing at nothing")
            # self.client.pub("kinect_pose", "nothing")
            return

        # 7. Extend the vector to the ends of the room
        extended_point = self.extend_vector_to_boundary(origin, vector)
        print(extended_point)

        topic = ""
        payload = ""
        # 8. Determine what the person is pointing at with some margin of error
        if (extended_point[0] == self.min[0]):
            print("Person is pointing at the right wall")
            # topic = "zigbee2mqtt/Ikea Light/set"
            # payload = '{"state": "OFF"}'
        elif (extended_point[0] == self.max[0]):
            print("Person is pointing at the left wall")
            # topic = "zigbee2mqtt/Ikea Light/set"
            # payload = '{"state": "OFF"}'
        elif (extended_point[1] == self.min[1]):
            print("Person is pointing at the floor")
            # topic = "zigbee2mqtt/Ikea Light/set"
            # payload = '{"state": "OFF"}'
        elif (extended_point[1] == self.max[1]):
            print("Person is pointing at the ceiling")
            print("Toggle light")
            topic = "zigbee2mqtt/Ikea Light/set"
            payload = '{"state": "TOGGLE"}'
        elif (extended_point[2] == self.min[2]):
            print("Person is pointing at the front wall")
            topic = "zigbee2mqtt/IR Emitter/set"
            payload = '{"ir_code_to_send": "C38JNwLVBDcCgAI3AuAHB+ADE0ALgAEBnGTgGzPAAUA3AYAC4BUzQAHAewmAAjcCgAI3AoAC"}'
        elif (extended_point[2] == self.max[2]):
            print("Person is pointing at the back wall")
            # topic = "zigbee2mqtt/Ikea Light/set"
            # payload = '{"state": "OFF"}'
        
        if (topic != "" and payload != ""):
            self.client.pub(topic, payload)
            self.frames_since_last_pub = 0
    
    def min_depth(self, depth, keypoint):
        diameter = 10
        x, y = map(round, keypoint)
        min_depth = math.inf

        for dy in range(-diameter//2, diameter//2 + 1):
            for dx in range(-diameter//2, diameter//2 + 1):
                if (0 <= x + dx < depth.shape[1] and
                    0 <= y + dy < depth.shape[0]):
                    d = depth[y + dy][x + dx]
                    if d != 0 and d < min_depth:
                        min_depth = d
        return min_depth

    def pixel_to_world(self, pixel, rect_matrix, depth):
        pixel_homogeneous = np.array([pixel[0], pixel[1], 1])
        world_coords = np.matmul(np.linalg.inv(rect_matrix), pixel_homogeneous)
        world_coords *= depth
        world_coords[1] *= -1 # make y positive up
        return world_coords
    
    def extend_vector_to_boundary(self, point, vector):
        # Calculate intersection points with each boundary
        t_values = []
        
        for i in range(3):
            if vector[i] != 0:
                t_min = (self.min[i] - point[i]) / vector[i]
                t_max = (self.max[i] - point[i]) / vector[i]
                t_values.extend([t_min, t_max])
        
        # for debugging when t_values is empty
        # print("DEBUG:\n  Point : " +  point + "\n  Vector: " + vector)

        t_values = [t for t in t_values if t > 0]

        # Find the smallest positive t-value
        t_min = min(t_values)

        # Extend the vector to the boundary
        extended_point = point + t_min * vector
        return extended_point
    
    def vector_is_valid(self, vector):
        return np.isfinite(vector).all() and np.any(vector != 0)
    
    def arm_is_extended(self, distance, threshold):
        return np.isfinite(distance) and distance > threshold

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