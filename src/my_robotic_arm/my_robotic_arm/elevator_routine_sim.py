import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import Constraints, PositionConstraint, BoundingVolume, JointConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
import copy
import time

class ElevatorButtonSimulator(Node):
    def __init__(self):
        super().__init__('elevator_routine_simulator')
        
        # Action & Service Clients
        self._move_action = ActionClient(self, MoveGroup, 'move_action')
        self._cartesian_srv = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self._execute_action = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')

        # Joint Names from your URDF
        self.joint_names = [
            'j1_arm_joint', 'j2_arm_joint', 'j3_arm_joint', 
            'j4_arm_joint', 'j5_arm_joint', 'j6_arm_joint'
        ]
        
        # Your floating Home position in radians
        self.home_angles = [0.0, -0.50, 1.20, -0.70, 0.0, 0.0]
        self.move_group_name = "arm" 

    def execute_test(self, button_x, button_y, button_z):
        self.get_logger().info('================================')
        self.get_logger().info('   GAZEBO ELEVATOR BUTTON TEST  ')
        self.get_logger().info('================================')

        self.get_logger().info('Phase 0: Moving to floating Home position...')
        if not self.move_to_home():
            self.get_logger().error('Failed to initialize at Home.')
            return
        time.sleep(1.0)

        # Pre-calculate the hover pose for the Recovery Block
        hover_pose = Pose()
        hover_pose.position.x = button_x - 0.05 # Hover 5cm in front of the button
        hover_pose.position.y = button_y
        hover_pose.position.z = button_z
        
        # Quaternion pointing straight forward along the X-axis
        hover_pose.orientation.x = 0.0
        hover_pose.orientation.y = 0.0
        hover_pose.orientation.z = 0.0
        hover_pose.orientation.w = 1.0

        try:
            # PHASE 1: HOVER
            self.get_logger().info('Phase 1: Aligning 5cm in front of the button...')
            if not self.move_to_pose(hover_pose, enforce_orientation=False):
                raise RuntimeError("Phase 1 (Hover) Failed. Target out of reach.")
            time.sleep(1.0) 

            # [IN REAL INTEGRATION: PUBLISH "MODE:STIFF" TO ESP32 HERE]
            self.get_logger().info('HARDWARE LOCK: Mode set to STIFF for Stall Detection.')

            # PHASE 2: THE LINEAR PUSH (Cartesian Path)
            self.get_logger().info('Phase 2: Executing perfectly straight Cartesian push...')
            
            # We command the arm to go 1cm PAST the button's surface.
            push_target = copy.deepcopy(hover_pose)
            push_target.position.x = button_x + 0.01 
            
            waypoints = [hover_pose, push_target]

            req = GetCartesianPath.Request()
            req.header.frame_id = 'base_link'
            req.group_name = self.move_group_name
            req.link_name = 'j6_arm_link'
            req.waypoints = waypoints
            req.max_step = 0.01 

            self._cartesian_srv.wait_for_service()
            future = self._cartesian_srv.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            
            if response.fraction < 0.9:
                raise RuntimeError(f"Phase 2 (Push) Failed. Only solved {response.fraction * 100}%. Kinematic lock.")
                
            self.execute_trajectory(response.solution)
            
            # Simulate the physical press duration
            time.sleep(0.5) 
            self.get_logger().info('Button Pressed! (Physical ESP32 Stall would trigger here).')

            # PHASE 3: SAFE RETRACT
            self.get_logger().info('Phase 3: Retracting from button panel...')
            retract_pose = copy.deepcopy(push_target)
            retract_pose.position.x -= 0.10 # Pull back 10cm to clear the area
            
            if not self.move_to_pose(retract_pose, enforce_orientation=True):
                 raise RuntimeError("Phase 3 (Retract) Failed.")
            time.sleep(1.0)

        except RuntimeError as e:
            # === THE RECOVERY BLOCK (FIXED) ===
            self.get_logger().error(f"ABORT TRIGGERED: {e}")
            self.get_logger().warn("Executing Safe Retract using saved hover position...")
            
            # Use the pre-calculated hover_pose to pull straight back
            retreat_pose = copy.deepcopy(hover_pose)
            retreat_pose.position.x -= 0.10 # Pull 10cm further back from the hover spot
            self.move_to_pose(retreat_pose, enforce_orientation=False)
            
        finally:
            # === THE HOME BLOCK ===
            self.get_logger().info('Phase 4: Folding back into Home position...')
            self.move_to_home()
            self.get_logger().info('Routine Complete!')

    def move_to_home(self):
        self._move_action.wait_for_server()
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.move_group_name
        
        constraints = Constraints()
        for i, j_name in enumerate(self.joint_names):
            jc = JointConstraint()
            jc.joint_name = j_name
            jc.position = self.home_angles[i]
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
            
        goal_msg.request.goal_constraints.append(constraints)
        
        future = self._move_action.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted: return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return True

    def move_to_pose(self, target_pose, enforce_orientation=False):
        self._move_action.wait_for_server()
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.move_group_name
        
        constraints = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = "base_link" 
        pc.link_name = "j6_arm_link"    
        s = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01])
        bv = BoundingVolume(primitives=[s], primitive_poses=[target_pose])
        pc.constraint_region = bv
        pc.weight = 1.0
        constraints.position_constraints.append(pc)
        
        if enforce_orientation:
            oc = OrientationConstraint()
            oc.header.frame_id = "base_link"
            oc.link_name = "j6_arm_link"
            oc.orientation = target_pose.orientation
            oc.absolute_x_axis_tolerance = 0.05 
            oc.absolute_y_axis_tolerance = 0.05
            oc.absolute_z_axis_tolerance = 0.05
            oc.weight = 1.0
            constraints.orientation_constraints.append(oc)
            
        goal_msg.request.goal_constraints.append(constraints)
        
        future = self._move_action.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted: return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return True

    def execute_trajectory(self, trajectory):
        self._execute_action.wait_for_server()
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        future = self._execute_action.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)


def main(args=None):
    rclpy.init(args=args)
    node = ElevatorButtonSimulator()
    
    # ADJUSTED: Moved closer (x=0.35) to prevent the arm from hitting a straight-elbow lock
    sim_x = 0.35 
    sim_y = 0.00
    sim_z = 0.35 
    
    node.execute_test(sim_x, sim_y, sim_z)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()