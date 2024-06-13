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
        # i know this data is paired as step/width is 2
        # but this function is generic and would work for rgb values too
        combined_data = []
        pixelInc = int(msg.step / msg.width)
        for i in range(0, len(msg.data), pixelInc):
            sum = 0
            if (msg.is_bigendian):
                for j in range(pixelInc):
                    sum += msg.data[i + j] << (8 * (pixelInc-1 - j))
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
        
        # width must be divisible by divisions x and
        # height must be divisible by divisions y
        divisions_x = 2
        divisions_y = 4
        compressed_data = []
        for y in range(0, len(data), divisions_y):
            row = []
            for x in range(0, len(data[y]), divisions_x):
                avg = 0
                for i in range(divisions_y):
                    for j in range(divisions_x):
                        for k in range(int(msg.step / msg.width)):
                            # take the 1st 8 bits, 2nd 8 bits, etc and take their averages
                            avg += int(((data[y + i][x + j] >> 8 * k) & 0b11111111) / (divisions_x * divisions_y)) << 8 * k
                row.append(avg)
            compressed_data.append(row)

        image_string = "\n"
        for y in range(len(compressed_data)):
            for x in range(len(compressed_data[y])):
                # you must do export RCUTILS_COLORIZED_OUTPUT=1 in the terminal before running
                image_string += '\x1b[48;2;%d;%d;%dm \x1b[0m' % ((compressed_data[y][x] >> 16) & 0b11111111, (compressed_data[y][x] >> 8) & 0b11111111, compressed_data[y][x] & 0b11111111)
            image_string += '\n'

        self.get_logger().info(image_string)


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