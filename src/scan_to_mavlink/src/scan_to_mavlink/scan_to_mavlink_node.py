import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from scan_to_mavlink_interfaces.msg import ObstacleDistance
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
            '/uas1/obstacle_distance',
            10)
        self.get_logger().info('Publishing OBSTACLE_DISTANCE to /uas1/obstacle_distance')

    def scan_callback(self, msg):
        dist_msg = ObstacleDistance()
        dist_msg.header = msg.header
        dist_msg.min_distance = 50  # cm
        dist_msg.max_distance = 1000  # cm
        dist_msg.increment = 2       # degrees per bin
        dist_msg.angle_offset = 0
        dist_msg.frame = 0
        dist_msg.sensor_type = 0

        num_values = 72
        step = max(1, len(msg.ranges) // num_values)
        distances = []

        for i in range(0, len(msg.ranges), step):
            d = msg.ranges[i]
            if math.isinf(d) or math.isnan(d):
                cm = 0
            else:
                cm = int(d * 100)
                if cm < 50 or cm > 1000:
                    cm = 0
            distances.append(cm)
            if len(distances) >= num_values:
                break

        while len(distances) < num_values:
            distances.append(0)

        dist_msg.distances = distances
        self.publisher_.publish(dist_msg)

def main():
    rclpy.init()
    node = ScanToMavlinkNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
