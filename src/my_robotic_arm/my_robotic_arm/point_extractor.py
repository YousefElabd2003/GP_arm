import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point # Added to send coordinates
import sensor_msgs_py.point_cloud2 as pc2

class PointExtractor(Node):

    def __init__(self):
        super().__init__('point_extractor')

        # 1. Subscriber to Stereo Camera
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.listener_callback,
            10)
        
        # 2. Publisher to the Arm Bridge
        self.publisher = self.create_publisher(Point, '/detected_object_3d', 10)
        self.get_logger().info('Stereo Point Publisher is running...')

    def listener_callback(self, msg):
        # Read points from the cloud
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

        if len(points) > 0:
            # For testing, we pick the middle point of the cloud
            # In a real scenario, you'd use a clustering algorithm here
            target_idx = len(points) // 2
            x, y, z = points[target_idx]

            # 3. Create the ROS 2 message
            target_msg = Point()
            # Note: You might need to offset these values depending on 
            # where your camera is mounted relative to the 'base_link'
            target_msg.x = float(x)
            target_msg.y = float(y)
            target_msg.z = float(z)

            # 4. Publish to the Arm Bridge script
            self.publisher.publish(target_msg)
            
            # Throttle logs so they don't flood your Core Ultra 9 terminal
            self.get_logger().info(f'Publishing Target: X={x:.3f}, Y={y:.3f}, Z={z:.3f}', throttle_duration_sec=1.0)

def main():
    rclpy.init()
    node = PointExtractor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()