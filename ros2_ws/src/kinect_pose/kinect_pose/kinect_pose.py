from enum import IntEnum
from ultralytics import YOLO
from .MqttClient import MqttClient
import torch
import math
import numpy as np
import rclpy
import json
import re
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
from playsound import playsound
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
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
        self.min = [-2000, -1000, 0000]
        self.max = [ 2000,  2000, 5000]

        self.model = YOLO("assets/models/yolov8m-pose.pt")
        self.model = self.model.to("cuda" if torch.cuda.is_available() else "cpu")

        # using HD camera, needs calibrating
        color_image = Subscriber(self, Image, "/kinect2/hd/image_color_rect")
        depth_image = Subscriber(self, Image, "/kinect2/hd/image_depth_rect")
        camera_info = Subscriber(self, CameraInfo, "/kinect2/hd/camera_info")
        self.create_subscription(String, '/speech_to_text', self.process_speech, 10)

        self.tss = TimeSynchronizer([color_image, depth_image, camera_info], 10)
        self.tss.registerCallback(self.image_callback)

        self.publisher = self.create_publisher(String, '/gesture_command', 10)

        self.client = MqttClient()
        self.frames_holding_gesture = 0
        self.last_gesture = ""

        self.gesture_history = []
        self.load_commands()
        
        event_handler = CommandFileHandler(self)
        observer = Observer()
        observer.schedule(event_handler, path="assets/json/commands.json", recursive=False)
        observer.start()

    def image_callback(self, image, depth, info):
        stream          = bridge.imgmsg_to_cv2(image, "bgr8"       )
        depth           = bridge.imgmsg_to_cv2(depth, "passthrough")
        rect_matrix     = np.array(info.k).reshape(3, 3)
        results         = self.model(source=stream, show=True)
        gesture_time    = self.get_clock().now().nanoseconds
        isPerson        = results[0].keypoints.has_visible
        result_keypoint = results[0].keypoints.xy.cpu().numpy()[0]

        left_gesture, right_gesture, left_distance, right_distance = self.determine_gesture(depth, rect_matrix, result_keypoint, isPerson)
        self.store_gesture(left_gesture, right_gesture, left_distance, right_distance, gesture_time)
        # self.send_gesture(left_gesture, right_gesture)

    def determine_gesture(self, depth, rect_matrix, result_keypoint, isPerson):
        if not isPerson:
            return "none", "none", 0, 0

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
        
        # 6. Determine if arm is extended
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
        #     print("Using shoulder dist")
        # else:
        #     print("Using hip dist")

        # 7. Determine user's gesture
        left_gesture = "none"
        right_gesture = "none"
        if self.arm_is_extended(left_distance, threshold) and self.vector_is_valid(left_vector):
            left_gesture = self.pointing_at(left_wrist_3d, left_vector)
        if self.arm_is_extended(right_distance, threshold) and self.vector_is_valid(right_vector):
            right_gesture = self.pointing_at(right_wrist_3d, right_vector)

        return left_gesture, right_gesture, left_distance, right_distance

    # def send_gesture(self, left_gesture, right_gesture):
    #     if left_gesture == "none" and right_gesture == "none":
    #         return

    #     # 8. Prepare gesture message
    #     topic = "kinect_pose"
    #     gesture = f"left_arm_{left_gesture}_right_arm_{right_gesture}"
    #     if left_gesture == "none":
    #         gesture = f"right_arm_{right_gesture}"
    #     elif right_gesture == "none":
    #         gesture = f"left_arm_{left_gesture}"

    #     payload = f'{{"gesture": "{gesture}"}}'

    #     # 9. Send gesture message
    #     # 9a. If new gesture, reset timer, needs to be held for 2 frames
    #     if self.last_gesture != gesture:
    #         print(f"old gesture: {self.last_gesture} new gesture: {gesture}")
    #         self.frames_holding_gesture = 1
    #         self.last_gesture = gesture
    #         return

    #     # 9b. If gesture has been held for less than 2 frames, increment
    #     if self.frames_holding_gesture < 2:
    #         print(f"frames holding gesture: {str(self.frames_holding_gesture)}")
    #         self.frames_holding_gesture += 1
    #         return

    #     # 9c. Otherwise, send the message
    #     self.client.pub(topic, payload)
    #     self.publisher.publish(String(data=payload))
    #     print(f"Sending message: {payload}")

    def store_gesture(self, left_gesture, right_gesture, left_distance, right_distance, gesture_time):
        # only store the last 10 seconds of gestures
        self.gesture_history = [gesture for gesture in self.gesture_history if gesture["time"] > gesture_time - 10000000000]

        # don't store if both gestures are none
        if left_gesture == "none" and right_gesture == "none":
            return

        # 7b. Determine single gesture
        gesture = ""
        if left_gesture != "none" and right_gesture != "none":
            if left_distance > right_distance:
                gesture = left_gesture
            else:
                gesture = right_gesture
        else:
            if left_gesture != "none":
                gesture = left_gesture
            else:
                gesture = right_gesture

        # 8. store gesture message
        if len(self.gesture_history) < 2:
            self.gesture_history.append({"time": gesture_time, "gesture": gesture})
            return

        last = self.gesture_history[-1]
        prev = self.gesture_history[-2]
        # new gesture is closer to 250ms since the last gesture
        if abs(gesture_time - prev["time"] - 250000000) < abs(last["time"] - prev["time"] - 250000000):
            self.gesture_history[-1] = {"time": gesture_time, "gesture": gesture}
        # it has been more than 250ms since the last gesture this gesture is not closer to the 250ms mark
        else:
            self.gesture_history.append({"time": gesture_time, "gesture": gesture})

    def process_speech(self, speech_info):
        # 9. Process speech and get command
        speech_json = json.loads(speech_info.data)
        print(json.dumps(speech_json, indent=2))

        topic = "kinect_pose"
        start_time = speech_json["start_time_ns"]
        payload = ""
        send_command = False
        starts_with_home = False

        for sentence in speech_json["alternatives"]:
            text = sentence["text"]
            if text.startswith("home"):
                starts_with_home = True

        for sentence in speech_json["alternatives"]:
            possible_matches = []

            for device in self.commands.keys():
                for command in self.commands[device]:
                    regex = rf".*\b{command}\b.*\bthat\b.*\b{device}\b.*"
                    regex_all = rf".*\b{command}\b.*\b(all|every)\b.*{device}s?\b.*"
                    if re.match(regex_all, text):
                        possible_matches.append({
                            "command": command,
                            "device": device,
                            "all": True,
                            "time": self.word_time(device, sentence),
                            "sentence": sentence
                        })
                    elif re.match(regex, text):
                        possible_matches.append({
                            "command": command,
                            "device": device,
                            "all": False,
                            "time": self.word_time(device, sentence),
                            "sentence": sentence
                        })
            
            if len(possible_matches) == 0:
                continue

            possible_matches.sort(key=lambda x: x["time"], reverse=True)
            closest_match = possible_matches[0]
            send_command = True
            payload = self.get_command(closest_match, start_time)
            break

        # 10. Send command message
        if not send_command:
            if starts_with_home:
                self.respond("no_command")
            return
        
        self.client.pub(topic, payload)
        self.publisher.publish(String(data=payload))
        self.get_logger().info(f"Sending message: {payload}")
    
    def get_command(self, closest_match, start_time = 0):
        closest_match["command"] = closest_match["command"].replace(" ", "_")
        closest_match["device"] = closest_match["device"].replace(" ", "_")
        if (closest_match["all"]):
            return f'{{"command": "all_{closest_match["device"]}s.{closest_match["command"]}"}}'
        
        that_time = self.word_time("that", closest_match["sentence"], start_time)

        closest_gesture = ""
        max_diff = math.inf
        for gesture_time in self.gesture_history:
            diff = abs(gesture_time["time"] - that_time)
            if diff < max_diff:
                max_diff = diff
                closest_gesture = gesture_time["gesture"]

        if (closest_gesture == ""):
            self.respond("no_gesture")

        return f'{{"command": "{closest_gesture}_{closest_match["device"]}.{closest_match["command"]}"}}'


    def word_time(self, match_word, sentence, start_time = 0):
        word_relative_time = 0
        for word in sentence["result"]:
            if word["word"] == match_word:
                # add half a second to account for processing delay
                # 0.5s chosen based on preliminary testing
                word_relative_time = (word["start"] + 0.5) * 10 ** 9

        return start_time + word_relative_time

    def respond(self, reason):
        try:
            playsound(f"assets/audio/{reason}.mp3")
        except Exception as e:
            self.get_logger().warning(f"Error playing audio: {e}")

    def load_commands(self):
        with open("assets/json/commands.json", "r") as f:
            self.commands = json.load(f)
        self.get_logger().info(f"Commands reloaded:{json.dumps(self.commands, indent=2)}")

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

        t_values = [t for t in t_values if t > 0]

        # for debugging when t_values is empty
        t_min = 0
        try:
            t_min = min(t_values)
        except Exception as e:
            self.get_logger().debug(f"DEBUG:\n  Point : {np.array2string(point)}\n  Vector: {np.array2string(vector)}")
            raise e

        # Find the smallest positive t-value

        # Extend the vector to the boundary
        extended_point = point + t_min * vector
        # Round off small values
        extended_point[np.abs(extended_point) < 0.0001] = 0

        return extended_point

    def pointing_at(self, origin, vector):
        extended_point = self.extend_vector_to_boundary(origin, vector)

        if extended_point[0] == self.min[0]:
            return "right"
        elif extended_point[0] == self.max[0]:
            return "left"
        elif extended_point[1] == self.min[1]:
            return "floor"
        elif extended_point[1] == self.max[1]:
            return "ceiling"
        elif extended_point[2] == self.min[2]:
            return "front"
        elif extended_point[2] == self.max[2]:
            return "back"
        else:
            return "none"

    def vector_is_valid(self, vector):
        return np.isfinite(vector).all() and np.any(vector != 0)

    def arm_is_extended(self, distance, threshold):
        return np.isfinite(distance) and distance > threshold

class CommandFileHandler(FileSystemEventHandler):
    def __init__(self, node):
        self.node = node

    def on_modified(self, event):
        if event.src_path.endswith("commands.json"):
            self.node.load_commands()

def main(args=None):
    rclpy.init(args=args)
    read_kinect_pose = ReadKinectPose()
    rclpy.spin(read_kinect_pose)
    read_kinect_pose.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
