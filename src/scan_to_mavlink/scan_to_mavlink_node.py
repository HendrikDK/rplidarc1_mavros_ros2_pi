import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from mavros_msgs.msg import ObstacleDistance
import math

class ScanToMavlinkNode(Node):
    def __init__(self):
        super().__init__('scan_to_mavlink_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.publisher_ = self.create_publisher(
            ObstacleDistance,
            '/mavros/obstacle/send',
            10)
        self.get_logger().info('Publishing OBSTACLE_DISTANCE to /mavros/obstacle/send')

    def scan_callback(self, msg):
        dist_msg = ObstacleDistance()
        dist_msg.header = msg.header
        dist_msg.min_distance = 50  # cm
        dist_msg.max_distance = 1000  # cm
        dist_msg.increment = 2       # degrees per bin
        dist_msg.angle_offset = 0
        dist_msg.frame = 0           # egocentric, forward = 0 deg
        dist_msg.sensor_type = 0     # Unknown, can be 3 (LIDAR)
        dist_msg.distances = []

        num_values = 72
        step = max(1, len(msg.ranges) // num_values)

        for i in range(0, len(msg.ranges), step):
            d = msg.ranges[i]
            if math.isinf(d) or math.isnan(d):
                cm = 0
            else:
                cm = int(d * 100)
                if cm < dist_msg.min_distance or cm > dist_msg.max_distance:
                    cm = 0
            dist_msg.distances.append(cm)
            if len(dist_msg.distances) >= num_values:
                break

        while len(dist_msg.distances) < num_values:
            dist_msg.distances.append(0)

        self.publisher_.publish(dist_msg)

def main():
    rclpy.init()
    node = ScanToMavlinkNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
