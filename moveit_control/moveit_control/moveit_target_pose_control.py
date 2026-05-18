#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from rcl_interfaces.msg import ParameterDescriptor
import numpy as np
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from copy import deepcopy
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.action import MoveGroup
import time
from std_srvs.srv import Trigger
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import Bool, String
from control_msgs.action import FollowJointTrajectory
import math
from controller_manager_msgs.srv import SwitchController


TRAJ_CTRL = "scaled_joint_trajectory_controller"
VEL_CTRL  = "forward_velocity_controller"

class MoveitTargetPoseControl(Node):

    def __init__(self):
        super().__init__('moveit_target_pose_control')

        #################################################
        # Declare parameter 
        #################################################
        self.declare_parameter('use_dummy_waypoint', True)
        self.declare_parameter('start', True)
        self.declare_parameter('pause', False)
        self.declare_parameter('stop', False)
        self.declare_parameter('planning_group', "ur5e_robot")
        self.declare_parameter('end_effector_link', "tool0")

        self.declare_parameter('dummy_x', 0.283237)
        self.declare_parameter('dummy_y', 0.109891)
        self.declare_parameter('dummy_z', 0.337834)
        self.declare_parameter('dummy_roll', 3.142)
        self.declare_parameter('dummy_pitch', 0.0)
        self.declare_parameter('dummy_yaw', 1.5708)
        
        self.use_dummy_waypoint = self.get_parameter('use_dummy_waypoint').get_parameter_value().bool_value
        self.planning_group = self.get_parameter('planning_group').get_parameter_value().string_value
        self.end_effector_link = self.get_parameter('end_effector_link').get_parameter_value().string_value
        self.start = self.get_parameter('start').get_parameter_value().bool_value
        self.pause = self.get_parameter('pause').get_parameter_value().bool_value
        self.stop = self.get_parameter('stop').get_parameter_value().bool_value

        #################################################
        # Declare variables as None 
        #################################################
        self.joint_state = None
        self.target_pos = None
        self.pose_msg_00 = None
        self.pose_msg_01 = None
        self.pose_msg_02 = None
        self.tool_tip_reference = None      
        self._active_move_goal_handle = None
        self.x = None
        self.y = None
        self.z = None
        self.roll = None
        self.pitch = None
        self.yaw = None
        self.transforms_to_broadcast = []

        #################################################
        # Create callback groups
        #################################################
        # This allows service and action client to run concurrently
        self.service_cb_group = ReentrantCallbackGroup()
        self.action_cb_group = ReentrantCallbackGroup()
        
        #################################################
        # Setup Transform broadcaster
        #################################################
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_timer = self.create_timer(0.01, self.broadcast_transforms_timer_callback)
        
        #################################################
        # Moveit2 Control
        #################################################
        # IMPORTANT: Add callback_group to action client
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            '/move_action',
            callback_group=self.action_cb_group
        )
        
        self.get_logger().info('Waiting for MoveIt2 action server...')
        self.move_group_client.wait_for_server()
        self.get_logger().info('MoveIt2 action server connected!')

        #################################################
        # Service Server and Client
        #################################################
        self.switch_cli = self.create_client(
            SwitchController, 
            "/controller_manager/switch_controller"
            )

        self.go_to_pose_00_service = self.create_service(
            Trigger,
            'go_to_pose_00',
            self.go_to_pose_00_callback,
            callback_group=self.service_cb_group
        )

        self.go_to_pose_01_service = self.create_service(
            Trigger,
            'go_to_pose_01',
            self.go_to_pose_01_callback,
            callback_group=self.service_cb_group
        )

        self.go_to_pose_02_service = self.create_service(
            Trigger,
            'go_to_pose_02',
            self.go_to_pose_02_callback,
            callback_group=self.service_cb_group
        )

        self.get_logger().info('All services are ready!')

        #################################################
        # Subscribers and publishers
        #################################################
        self.create_subscription(
            JointState, 
            "/joint_states", 
            self.joint_state_callback, 
            10
            )
        
        self.start_subscriber = self.create_subscription(
            Bool,
            '/start',
            self.start_callback,
            10
            )

        self.pause_subscriber = self.create_subscription(
            Bool,
            '/pause',
            self.pause_callback,
            10
            )

        self.stop_subscriber = self.create_subscription(
            Bool,
            '/stop',
            self.stop_callback,
            10
            )  
        
        self.publisher = self.create_publisher(
            String, 
            'trajectory_execution_event', 
            10
            )


        if self.use_dummy_waypoint:

            self.x = self.get_parameter('dummy_x').get_parameter_value().double_value
            self.y = self.get_parameter('dummy_y').get_parameter_value().double_value
            self.z = self.get_parameter('dummy_z').get_parameter_value().double_value

            self.roll = self.get_parameter('dummy_roll').get_parameter_value().double_value
            self.pitch = self.get_parameter('dummy_pitch').get_parameter_value().double_value
            self.yaw = self.get_parameter('dummy_yaw').get_parameter_value().double_value

            self.quat = quaternion_from_euler(self.roll, self.pitch, self.yaw)
            self.compute_waypoints(self.x, self.y, self.z, self.quat[0], self.quat[1], self.quat[2], self.quat[3])

        else:
            qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,  # ✅ cache last 10
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                reliability=QoSReliabilityPolicy.RELIABLE,
            )

            self.pose_subscriber = self.create_subscription(
                PoseStamped, 
                '/pose', 
                self.pose_callback, 
                qos
                )

    #################################################
    # Client Function
    #################################################
    def switch_controller(self, activate, deactivate, strictness=1):
        if not self.switch_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("switch_controller service not available!")
            return False

        req = SwitchController.Request()
        req.activate_controllers = list(activate)
        req.deactivate_controllers = list(deactivate)
        req.strictness = strictness

        future = self.switch_cli.call_async(req)

        # Poll instead of spin_until_future_complete — avoids deadlock with MultiThreadedExecutor
        timeout = 5.0
        start_time = time.time()
        while not future.done():
            if (time.time() - start_time) > timeout:
                self.get_logger().error("switch_controller timed out!")
                return False
            time.sleep(0.01)

        if future.result() is None:
            self.get_logger().error("switch_controller service call failed.")
            return False

        ok = future.result().ok
        self.get_logger().info(f"switch_controller ok={ok}, activate={activate}, deactivate={deactivate}")
        return ok


    #################################################
    # Callbacks
    #################################################

    def start_callback(self, msg: Bool):        
        self.start = msg.data

    def pause_callback(self, msg: Bool):        
        self.pause = msg.data

    def stop_callback(self, msg: Bool):        
        self.stop = msg.data
    
    def joint_state_callback(self, msg: JointState):        
        self.joint_state = msg

    def pose_callback(self, msg):
        
        #IMPORTANT: THE VALUE IS BEING EDITTED TO A CUSTOM APPLICATION
        self.current_pose = msg
        self.x = self.current_pose.pose.position.x
        self.y = self.current_pose.pose.position.y
        self.z = self.current_pose.pose.position.z
        # quat = [
        #     self.current_pose.pose.orientation.x,
        #     self.current_pose.pose.orientation.y,
        #     self.current_pose.pose.orientation.z, 
        #     self.current_pose.pose.orientation.w
        #     ]
        
        angle_x = math.pi / 2      # 90° around X
        angle_z = -math.pi / 2      # 90° around Z

        # Quaternion for X+90°
        x1 = math.sin(angle_x / 2)  # ≈ 0.7071
        y1 = 0.0
        z1 = 0.0
        w1 = math.cos(angle_x / 2)  # ≈ 0.7071

        # Quaternion for Z-90° (anti-clockwise = negative rotation) (not sure what is going but it works for now so not gonna touch it)
        x2 = 0.0
        y2 = -math.sin(angle_z / 2) 
        z2 = 0.0 # ≈ -0.7071  <-- negated for anti-clockwise
        w2 = math.cos(angle_z / 2)   # ≈  0.7071
        
        # Combine: q_combined = q_z180 * q_x90  (right-to-left application)
        xc = w2*x1 + x2*w1 + y2*z1 - z2*y1
        yc = w2*y1 - x2*z1 + y2*w1 + z2*x1
        zc = w2*z1 + x2*y1 - y2*x1 + z2*w1
        wc = w2*w1 - x2*x1 - y2*y1 - z2*z1

        # Apply combined rotation to current pose quaternion
        new_x = self.current_pose.pose.orientation.w*xc + self.current_pose.pose.orientation.x*wc + self.current_pose.pose.orientation.y*zc - self.current_pose.pose.orientation.z*yc
        new_y = self.current_pose.pose.orientation.w*yc - self.current_pose.pose.orientation.x*zc + self.current_pose.pose.orientation.y*wc + self.current_pose.pose.orientation.z*xc
        new_z = self.current_pose.pose.orientation.w*zc + self.current_pose.pose.orientation.x*yc - self.current_pose.pose.orientation.y*xc + self.current_pose.pose.orientation.z*wc
        new_w = self.current_pose.pose.orientation.w*wc - self.current_pose.pose.orientation.x*xc - self.current_pose.pose.orientation.y*yc - self.current_pose.pose.orientation.z*zc
        quat = [
            new_x,
            new_y,
            new_z,
            new_w
            ]

        self.compute_waypoints(self.x, self.y, self.z, quat[0], quat[1], quat[2], quat[3])

   
    def go_to_pose_00_callback(self, request, response):

        if self.pose_msg_00 is None or self.pose_msg_01 is None:
            response.success = False
            response.message = "Waypoints not generated yet!"
            self.get_logger().warn(response.message)
            return response
        
        self.get_logger().info('Service called: Executing path...')

        ok = self.switch_controller(activate=[TRAJ_CTRL], deactivate=[VEL_CTRL], strictness=1)
        if not ok:
            return False
        time.sleep(0.5)
        success = self.generate_path(0)
        
        if success:
            response.success = True
            response.message = "Path executed successfully!"
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = "Path execution failed!"
            self.get_logger().error(response.message)
        
        return response
    
    def go_to_pose_01_callback(self, request, response):

        if self.pose_msg_00 is None or self.pose_msg_01 is None:
            response.success = False
            response.message = "Waypoints not generated yet!"
            self.get_logger().warn(response.message)
            return response
        
        self.get_logger().info('Service called: Executing path...')
        
        ok = self.switch_controller(activate=[TRAJ_CTRL], deactivate=[VEL_CTRL], strictness=1)
        if not ok:
            return False
        time.sleep(0.5)
        success = self.generate_path(1)
        
        if success:
            response.success = True
            response.message = "Path executed successfully!"
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = "Path execution failed!"
            self.get_logger().error(response.message)
        
        return response
    
    def go_to_pose_02_callback(self, request, response):

        if self.pose_msg_00 is None or self.pose_msg_01 is None:
            response.success = False
            response.message = "Waypoints not generated yet!"
            self.get_logger().warn(response.message)
            return response
        
        self.get_logger().info('Service called: Executing path...')

        ok = self.switch_controller(activate=[TRAJ_CTRL], deactivate=[VEL_CTRL], strictness=1)
        if not ok:
            return False
        time.sleep(0.5)
        success = self.generate_path(2)
        
        if success:
            response.success = True
            response.message = "Path executed successfully!"
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = "Path execution failed!"
            self.get_logger().error(response.message)
        
        return response

    def broadcast_transforms_timer_callback(self):

        if self.transforms_to_broadcast:
            for transform in self.transforms_to_broadcast:
                transform.header.stamp = self.get_clock().now().to_msg()
                self.tf_broadcaster.sendTransform(transform)
        else:
            print('transform_not_found')

    #################################################
    # Helper Functions
    #################################################
    def compute_waypoints(self, x, y, z, o_x, o_y, o_z, o_w):

        self.pose_msg_00 = PoseStamped()
        self.pose_msg_01 = PoseStamped()
        self.pose_msg_02 = PoseStamped()
        
        # pose_01
        self.pose_msg_01.header.stamp = self.get_clock().now().to_msg()
        self.pose_msg_01.header.frame_id = 'base_link'
        self.pose_msg_01.pose.position.x = x
        self.pose_msg_01.pose.position.y = y
        self.pose_msg_01.pose.position.z = z
        self.pose_msg_01.pose.orientation.x = o_x
        self.pose_msg_01.pose.orientation.y = o_y
        self.pose_msg_01.pose.orientation.z = o_z
        self.pose_msg_01.pose.orientation.w = o_w
        
        # pose_00
        self.pose_msg_00.header.stamp = self.get_clock().now().to_msg()
        self.pose_msg_00.header.frame_id = 'pose_01'
        self.pose_msg_00.pose.position.x = 0.0
        self.pose_msg_00.pose.position.y = 0.0
        self.pose_msg_00.pose.position.z = -0.05
        self.pose_msg_00.pose.orientation.x = 0.0
        self.pose_msg_00.pose.orientation.y = 0.0
        self.pose_msg_00.pose.orientation.z = 0.0
        self.pose_msg_00.pose.orientation.w = 1.0

        # pose_02
        self.pose_msg_02 = deepcopy(self.pose_msg_00)
        self.pose_msg_02.header.stamp = self.get_clock().now().to_msg()
        self.pose_msg_02.header.frame_id = 'pose_01'
        self.pose_msg_02.pose.position.z = 0.03

        self.boradcast_to_tf(self.pose_msg_02, 'pose_02')
        self.boradcast_to_tf(self.pose_msg_01, 'pose_01')
        self.boradcast_to_tf(self.pose_msg_00, 'pose_00')

        # # Tool tip reference Pose
        # self.tool_tip_reference = PoseStamped()
        # self.tool_tip_reference.header.stamp = self.get_clock().now().to_msg()
        # self.tool_tip_reference.header.frame_id = 'tool0'
        # self.tool_tip_reference.pose.position.x = 0.0
        # self.tool_tip_reference.pose.position.y = 0.0
        # self.tool_tip_reference.pose.position.z = 0.40
        # self.tool_tip_reference.pose.orientation.x = 0.0
        # self.tool_tip_reference.pose.orientation.y = 0.0
        # self.tool_tip_reference.pose.orientation.z = 0.0
        # self.tool_tip_reference.pose.orientation.w = 1.0
        # self.boradcast_to_tf(self.tool_tip_reference, 'tool_tip_reference')

    def boradcast_to_tf(self, pose_msg, child_frame_name):

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()

        t.header.frame_id = pose_msg.header.frame_id
        t.child_frame_id = child_frame_name
        t.transform.translation.x = pose_msg.pose.position.x
        t.transform.translation.y = pose_msg.pose.position.y
        t.transform.translation.z = pose_msg.pose.position.z
        t.transform.rotation.x = pose_msg.pose.orientation.x
        t.transform.rotation.y = pose_msg.pose.orientation.y
        t.transform.rotation.z = pose_msg.pose.orientation.z
        t.transform.rotation.w = pose_msg.pose.orientation.w
        
        self.transforms_to_broadcast = [
            tf for tf in self.transforms_to_broadcast if tf.child_frame_id != child_frame_name]
        self.transforms_to_broadcast.append(t)
        self.tf_broadcaster.sendTransform(t)

    def generate_path(self, destinations = 0):

        if self.pose_msg_00 is None or self.pose_msg_01 is None:

            self.get_logger().error('Waypoints not generated yet!')
            return False
        
        self.get_logger().info('Starting path execution...')
        pose_00_base = self.transform_pose_to_base_link(self.pose_msg_00)
        pose_01_base = self.transform_pose_to_base_link(self.pose_msg_01)
        pose_02_base = self.transform_pose_to_base_link(self.pose_msg_02)

        if destinations == 0:
            waypoints = [
                ('pose_00', pose_00_base, "PTP"),
            ]

        elif destinations == 1:
            waypoints = [
                ('pose_01', pose_01_base, "LIN"),
            ]

        elif destinations == 2:
            waypoints = [
                ('pose_02', pose_02_base, "LIN"),
            ]
        else: return False

        for name, pose, planner_id in waypoints:
            self.get_logger().info(f'Moving to {name}...')
            success = self.move_to_pose(pose, planner_id = planner_id)
            
            if not success:
                self.get_logger().error(f'Failed to reach {name}!')
                return False
            
            self.get_logger().info(f'✓ Reached {name}')
            time.sleep(0.5)
        
        self.get_logger().info('✓ Path completed!')
        return True

    def transform_pose_to_base_link(self, pose_stamped):

        if pose_stamped.header.frame_id == 'base_link': return pose_stamped
        
        transform = self.tf_buffer.lookup_transform(
            'base_link', pose_stamped.header.frame_id,
            rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=5.0))
        
        from tf2_geometry_msgs import do_transform_pose_stamped
        return do_transform_pose_stamped(pose_stamped, transform)

    def move_to_pose(self, target_pose, pipeline_id = "pilz_industrial_motion_planner", planner_id = "LIN"):

        from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
        from shape_msgs.msg import SolidPrimitive
        
        goal_msg = MoveGroup.Goal()
        goal_msg.request.workspace_parameters.header.frame_id = "base_link"
        goal_msg.request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        goal_msg.request.start_state.is_diff = True
        goal_msg.request.group_name = self.planning_group
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 0.1
        goal_msg.request.pipeline_id = pipeline_id
        goal_msg.request.planner_id = planner_id

        
        constraints = Constraints()
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "base_link"
        pos_constraint.link_name = self.end_effector_link
        pos_constraint.weight = 1.0
        bounding_volume = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.001]
        bounding_volume.primitives.append(sphere)
        bounding_volume.primitive_poses.append(target_pose.pose)
        pos_constraint.constraint_region = bounding_volume
        
        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = "base_link"
        orient_constraint.link_name = self.end_effector_link
        orient_constraint.orientation = target_pose.pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.001
        orient_constraint.absolute_y_axis_tolerance = 0.001
        orient_constraint.absolute_z_axis_tolerance = 0.001
        orient_constraint.weight = 1.0
        
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(orient_constraint)
        goal_msg.request.goal_constraints.append(constraints)
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True
        
        self.get_logger().info(f'Sending goal: [{target_pose.pose.position.x:.3f}, '
                            f'{target_pose.pose.position.y:.3f}, {target_pose.pose.position.z:.3f}]')
        
        # Send goal asynchronously
        send_goal_future = self.move_group_client.send_goal_async(goal_msg)
        
        # Wait for goal acceptance with polling (no spin_until_future_complete!)
        timeout = 10.0
        start_time = time.time()
        while not send_goal_future.done():
            if (time.time() - start_time) > timeout:
                self.get_logger().error('Timeout waiting for goal acceptance')
                return False
            time.sleep(0.01)  # Small sleep to prevent busy waiting
        
        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False
        
        self._active_move_goal_handle = goal_handle
        self.get_logger().info('Goal accepted, executing...')
        
        # Get result asynchronously
        result_future = goal_handle.get_result_async()
        
        
        # Wait for result with polling (no spin_until_future_complete!)
        timeout = 60.0
        start_time = time.time()

        while not result_future.done():
            
            # Execution timeout
            if (time.time() - start_time) > timeout:
                self.get_logger().error('Execution timeout')
                return False
            
            # Pause signal Triggered
            if self.pause:    

                msg = String()
                msg.data = 'stop'
                self.publisher.publish(msg)
                self.get_logger().warn('Pause signal triggered, motion paused')

                while not self.start: pass

                # Resend the goal (to resume)
                send_goal_future = self.move_group_client.send_goal_async(goal_msg)
                
                while not send_goal_future.done():
                    if (time.time() - start_time) > timeout:
                        self.get_logger().error('Timeout waiting for goal acceptance')
                        return False
                    time.sleep(0.01)  # Small sleep to prevent busy waiting
                
                goal_handle = send_goal_future.result()
                if not goal_handle or not goal_handle.accepted:
                    self.get_logger().error('Resume failed')
                    return False
                
                self._active_move_goal_handle = goal_handle                
                result_future = goal_handle.get_result_async()

                self.get_logger().info('\033[92mMotion resumed\033[0m')

            # Stop signal Triggered
            if self.stop:
                
                self.get_logger().warn('Stop signal triggered, cancelling motion...')

                msg = String()
                msg.data = 'stop'
                self.publisher.publish(msg)

                self.stop = False
                
                if self._active_move_goal_handle is not None:

                    cancel_future = self._active_move_goal_handle.cancel_goal_async()
                    start = time.time()
                    while not cancel_future.done() and (time.time() - start) < 2.0:
                        time.sleep(0.01)
                    self._active_move_goal_handle = None
                
                return False
            
            time.sleep(0.001)  # Small sleep to prevent busy waiting

        result = result_future.result().result
        
        if result.error_code.val == result.error_code.SUCCESS:
            self.get_logger().info('✓ Motion completed successfully!')
            return True
        else:
            error_messages = {
                1: "SUCCESS",
                -1: "FAILURE",
                -2: "PLANNING_FAILED",
                -3: "INVALID_MOTION_PLAN",
                -4: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
                -5: "CONTROL_FAILED",
                -6: "UNABLE_TO_AQUIRE_SENSOR_DATA",
                -7: "TIMED_OUT",
                -10: "PREEMPTED",
                -11: "START_STATE_IN_COLLISION",
                -12: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
                -13: "GOAL_IN_COLLISION",
                -14: "GOAL_VIOLATES_PATH_CONSTRAINTS",
                -15: "GOAL_CONSTRAINTS_VIOLATED",
                -21: "INVALID_GROUP_NAME",
                -22: "INVALID_GOAL_CONSTRAINTS",
                -23: "INVALID_ROBOT_STATE",
                -24: "INVALID_LINK_NAME",
                -31: "INVALID_OBJECT_NAME",
                -32: "FRAME_TRANSFORM_FAILURE",
                -51: "NO_IK_SOLUTION",
            }
            error_name = error_messages.get(result.error_code.val, f"UNKNOWN_ERROR_{result.error_code.val}")
            self.get_logger().error(f'✗ Motion failed: {error_name} (code: {result.error_code.val})')
            return False

        

#################################################
# Main
#################################################
def main(args=None):
    rclpy.init(args=args)
    moveit_target_pose_control = MoveitTargetPoseControl()
    
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(moveit_target_pose_control)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        moveit_target_pose_control.get_logger().error(f'Exception in executor: {e}')
    finally:
        # Proper cleanup order
        executor.remove_node(moveit_target_pose_control)
        moveit_target_pose_control.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()