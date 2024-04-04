import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

def find_closest_value(target, values):
    closest = None
    min_difference = float('inf')

    for value in values:
        difference = abs(value - target)
        if difference < min_difference:
            min_difference = difference
            closest = value

    return closest

class ReadKinectDepth(Node):
    def __init__(self):
        super().__init__('read_kinect_depth')
        self.subscription = self.create_subscription(
            Image,
            '/kinect2/sd/image_depth',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # i know this data is paired as step/width is 2
        # but this function is generic and would work for rgb values too
        paired_data = []
        for i in range(0, len(msg.data), int(msg.step / msg.width)):
            sum = 0
            if (msg.is_bigendian):
                for j in range(int(msg.step / msg.width)):
                    sum += msg.data[i + j] << (8 * (int(msg.step / msg.width) - 1 - j))
            else:
                for j in range(int(msg.step / msg.width)):
                    sum += msg.data[i + j] << (8 * j)
            paired_data.append(sum)

        data = []
        for i in range(0, len(paired_data), msg.width):
            row = []
            for j in range(msg.width):
                row.append(paired_data[i + j])
            data.append(row)
        
        compressed_data = []
        for y in range(0, len(data), 4):
            row = []
            for x in range(0, len(data[y]), 4):
                sum = 0
                sum += data[y    ][x] + data[y    ][x + 1] + data[y    ][x + 2] + data[y    ][x + 3]
                sum += data[y + 1][x] + data[y + 1][x + 1] + data[y + 1][x + 2] + data[y + 1][x + 3]
                sum += data[y + 2][x] + data[y + 2][x + 1] + data[y + 2][x + 2] + data[y + 2][x + 3]
                sum += data[y + 3][x] + data[y + 3][x + 1] + data[y + 3][x + 2] + data[y + 3][x + 3]
                row.append(sum)
            compressed_data.append(row)
        
        max_dist = 0
        for y in range(0, len(compressed_data)):
            for x in range(0, len(compressed_data[y])):
                if (compressed_data[y][x] > max_dist):
                    max_dist = compressed_data[y][x]

        ascii_chars = " `.-':_,^=;><+!rc*/z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@"
        ascii_brightness = [0, 0.0751, 0.0829, 0.0848, 0.1227, 0.1403, 0.1559, 0.185, 0.2183, 0.2417, 0.2571, 0.2852, 0.2902, 0.2919, 0.3099, 0.3192, 0.3232, 0.3294, 0.3384, 0.3609, 0.3619, 0.3667, 0.3737, 0.3747, 0.3838, 0.3921, 0.396, 0.3984, 0.3993, 0.4075, 0.4091, 0.4101, 0.42, 0.423, 0.4247, 0.4274, 0.4293, 0.4328, 0.4382, 0.4385, 0.442, 0.4473, 0.4477, 0.4503, 0.4562, 0.458, 0.461, 0.4638, 0.4667, 0.4686, 0.4693, 0.4703, 0.4833, 0.4881, 0.4944, 0.4953, 0.4992, 0.5509, 0.5567, 0.5569, 0.5591, 0.5602, 0.5602, 0.565, 0.5776, 0.5777, 0.5818, 0.587, 0.5972, 0.5999, 0.6043, 0.6049, 0.6093, 0.6099, 0.6465, 0.6561, 0.6595, 0.6631, 0.6714, 0.6759, 0.6809, 0.6816, 0.6925, 0.7039, 0.7086, 0.7235, 0.7302, 0.7332, 0.7602, 0.7834, 0.8037, 0.9999]

        image_string = "\n"
        for y in range(len(compressed_data)):
            for x in range(len(compressed_data[y])):
                dist_perc = 1 - (compressed_data[y][x] / max_dist)
                closest = find_closest_value(dist_perc, ascii_brightness)
                char_index = ascii_brightness.index(closest)
                char = ascii_chars[char_index]
                image_string += char
            image_string += '\n'

        self.get_logger().info(image_string)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = ReadKinectDepth()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()