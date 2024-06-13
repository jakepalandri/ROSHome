from ultralytics import YOLO
import torch
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class ReadKinectColour(Node):
    def __init__(self):
        super().__init__('read_kinect_color')
        self.subscription = self.create_subscription(
            Image,
            '/kinect2/sd/image_color_rect',
            self.listener_callback,
            10)
        self.model = YOLO("yolov8m-pose.pt")
        # self.model = self.model.to("cuda" if torch.cuda.is_available() else "cpu")
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # i know this data is paired as step/width is 2
        # but this function is generic and would work for rgb values too
        combined_data = []
        pixelInc = int(msg.step / msg.width)
        for i in range(0, len(msg.data), pixelInc):
            sum = 0
            if (msg.is_bigendian):
                for j in range(pixelInc):
                    sum += msg.data[i + j] << (8 * (pixelInc - 1 - j))
            else:
                for j in range(pixelInc):
                    sum += msg.data[i + j] << (8 * j)
            combined_data.append(sum)
        
        data = []
        for i in range(0, len(combined_data), msg.width):
            row = []
            for j in range(msg.width):
                row.append(combined_data[i + j])
            data.append(row)

        rgb_data = []
        for row in data:
            new_row = []
            for value in row:
                new_val = [value >> 16 & 0b11111111, value >> 8 & 0b11111111, value & 0b11111111]
                new_row.append(new_val)
            rgb_data.append(new_row)

        rgb_data = np.array(rgb_data, dtype="uint8")

        results = self.model(rgb_data)
        print("Names: ", results[0].names)
        print("Keypoints: ", results[0].keypoints)
        print("Boxes: ", results[0].boxes)

def main(args=None):
    rclpy.init(args=args)

    read_kinect_colour = ReadKinectColour()

    rclpy.spin(read_kinect_colour)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    read_kinect_colour.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()