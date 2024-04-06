import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

# Open the kinect viewer to see how you are controlling it
# Run the turtle sim
# Run this node
# Move your hands in front of the kinect to turn the turtle

class ReadKinectDepth(Node):
    def __init__(self):
        super().__init__('read_kinect_depth')
        self.subscription = self.create_subscription(
            Image,
            '/kinect2/sd/image_depth',
            self.listener_callback,
            10)
        # self.subscription  # prevent unused variable warning
        # self.publisher = publisher

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

        closest_left = float('inf')
        for y in range(0, len(data)):
            for x in range (0, int(len(data[y]) / 2)):
                if (data[y][x] < closest_left and data[y][x] != 0):
                    closest_left = data[y][x]

        closest_right = float('inf')
        for y in range(0, len(data)):
            for x in range (int(len(data[y]) / 2), len(data[y])):
                if (data[y][x] < closest_right and data[y][x] != 0):
                    closest_right = data[y][x]
        
        closest = min(closest_left, closest_right)
        diff = closest_left - closest_right
        
        turn = 0
        movement_str = ""
        if (diff > 200):
            turn = 1
            movement_str = "Turning left, "
        elif (diff < -200):
            turn = -1
            movement_str = "Turning right, "
        else:
            movement_str = "Not turning, "
        
        direction = 0
        if (closest < 200):
            direction = 1
            movement_str += "moving forwards"
        elif (closest > 500):
            direction = -1
            movement_str += "moving backwards"
        else:
            movement_str += "stationary"


        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        msg = Twist()
        msg.linear.x = float(direction)
        msg.angular.z = float(turn)
        self.publisher_.publish(msg)
        self.get_logger().info("\n%s\n%d\n" % (movement_str, turn))

def main(args=None):
    rclpy.init(args=args)

    read_kinect_depth = ReadKinectDepth()

    rclpy.spin(read_kinect_depth)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    read_kinect_depth.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()