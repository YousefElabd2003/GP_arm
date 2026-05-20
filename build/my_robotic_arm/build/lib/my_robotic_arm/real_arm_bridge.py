import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class RealArmBridge(Node):
    def __init__(self):
        super().__init__('real_arm_bridge')
        
        # Action client for the real hardware's MoveGroup
        self._action_client = ActionClient(self, MoveGroup, 'move_group')
        
        # Subscriber to listen for goals from your other scripts
        self.goal_sub = self.create_subscription(
            PoseStamped,
            'cmd_arm_pose',
            self.goal_callback,
            10)
        
        self.get_logger().info('Hardware Bridge Live. Listening on /cmd_arm_pose')

    def goal_callback(self, msg):
        self.get_logger().info(f'Received new goal for real robot: {msg.pose.position}')
        self.send_moveit_goal(msg)

    def send_moveit_goal(self, pose_msg):
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Real Robot MoveGroup not found!')
            return

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm"
        
        # Use the pose received from the other script
        goal_msg.request.goal_constraints = self.create_constraints(pose_msg)
        
        # Safety: On real hardware, always start with slower velocities
        goal_msg.request.max_velocity_scaling_factor = 0.1 
        goal_msg.request.max_acceleration_scaling_factor = 0.1

        self.get_logger().info('Executing movement on REAL hardware...')
        self._action_client.send_goal_async(goal_msg)

    def create_constraints(self, pose_msg):
        # (This is the same constraint logic used in your mover script)
        from moveit_msgs.msg import Constraints, PositionConstraint, BoundingVolume
        from shape_msgs.msg import SolidPrimitive
        
        constraints = Constraints()
        pc = PositionConstraint()
        pc.header = pose_msg.header
        pc.link_name = "j6_arm_link"
        
        s = SolidPrimitive()
        s.type = SolidPrimitive.SPHERE
        s.dimensions = [0.01]
        
        bv = BoundingVolume()
        bv.primitives.append(s)
        bv.primitive_poses.append(pose_msg.pose)
        
        pc.constraint_region = bv
        pc.weight = 1.0
        constraints.position_constraints.append(pc)
        return [constraints]

def main():
    rclpy.init()
    node = RealArmBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()