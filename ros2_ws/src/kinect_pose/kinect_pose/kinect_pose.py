from ultralytics import YOLO
import torch
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
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        return


def main(args=None):
    rclpy.init(args=args)

    model = YOLO("yolov8m-pose.pt")
    model = model.to("cuda" if torch.cuda.is_available() else "cpu")

    read_kinect_colour = ReadKinectColour()

    rclpy.spin(read_kinect_colour)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    read_kinect_colour.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()