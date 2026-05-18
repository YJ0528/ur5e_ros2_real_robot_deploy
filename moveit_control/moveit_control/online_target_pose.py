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
from tf_transformations import quaternion_matrix, quaternion_from_matrix

TRAJ_CTRL = "scaled_joint_trajectory_controller"
POS_CTRL  = "forward_position_controller"

class OnlineTargetPose(Node):

    def __init__(self):
        super().__init__('online_target_pose')

        #################################################
        # Declare parameter 
        #################################################
        self.declare_parameter('use_dummy_waypoint', True)
        self.declare_parameter('planning_group', "ur5e_robot")
        self.declare_parameter('end_effector_link', "tool0")
        self.declare_parameter('target_pose', "motion_tracking_pose")
        self.declare_parameter('_orbit_radius', 0.1)
        self.declare_parameter('_orbit_speed', 0.2)

        self.declare_parameter('dummy_x', 0.383237)
        self.declare_parameter('dummy_y', 0.109891)
        self.declare_parameter('dummy_z', 0.237834)
        self.declare_parameter('dummy_roll', 3.142)
        self.declare_parameter('dummy_pitch', 0.0)
        self.declare_parameter('dummy_yaw', 1.5708)

        self.use_dummy_waypoint = self.get_parameter('use_dummy_waypoint').get_parameter_value().bool_value
        self.planning_group = self.get_parameter('planning_group').get_parameter_value().string_value
        self.end_effector_link = self.get_parameter('end_effector_link').get_parameter_value().string_value
        self.target_pose = self.get_parameter('target_pose').get_parameter_value().string_value
        self._orbit_radius = self.get_parameter('_orbit_radius').get_parameter_value().double_value
        self._orbit_speed = self.get_parameter('_orbit_speed').get_parameter_value().double_value

        #################################################
        # Declare variables as None 
        #################################################
        self.pose_msg_00 = None
        self.pose_msg_01 = None
        self.pose_msg_02 = None
        self.pose_msg_03 = None
        
        self.x = None
        self.y = None
        self.z = None
        self.roll = None
        self.pitch = None
        self.yaw = None

        self._pose_01_base_x = None
        self._pose_01_base_y = None
        self._pose_01_base_z = None

        self._orbit_angle = math.pi 

        self._target_pose_offset = None
        self.transforms_to_broadcast = []
        
        #################################################
        # Setup Transform broadcaster
        #################################################
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_timer = self.create_timer(0.01, self.broadcast_transforms_timer_callback)

        #################################################
        # Service Server and Client
        #################################################
        self.switch_cli = self.create_client(
            SwitchController, 
            "/controller_manager/switch_controller"
            )

        self.get_logger().info('All services are ready!')

        #################################################
        # Subscribers and publishers
        #################################################        
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
            # self.compute_waypoints(self.x, self.y, self.z, self.quat[0], self.quat[1], self.quat[2], self.quat[3])

            qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,  # ✅ cache last 10
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                reliability=QoSReliabilityPolicy.RELIABLE,
            )

            self.target_pose_publisher = self.create_publisher(
                PoseStamped, 
                '/target_pose', 
                qos
                )
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
                
        self.get_logger().info('All topics are ready!')
        

    #################################################
    # Service Client Function
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
    def pose_callback(self, msg):
        
        # IMPORTANT: THE VALUE IS BEING EDITTED TO A CUSTOM APPLICATION
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
        
        angle = math.pi / 2
        x1 = math.sin(angle / 2)  
        y1 = 0.0
        z1 = 0.0
        w1 = math.cos(angle / 2) 
        
        # Quaternion multiplication: q_new = q_rot * q_current
        new_x = self.current_pose.pose.orientation.w*x1 + self.current_pose.pose.orientation.x*w1 + self.current_pose.pose.orientation.y*z1 - self.current_pose.pose.orientation.z*y1
        new_y = self.current_pose.pose.orientation.w*y1 - self.current_pose.pose.orientation.x*z1 + self.current_pose.pose.orientation.y*w1 + self.current_pose.pose.orientation.z*x1
        new_z = self.current_pose.pose.orientation.w*z1 + self.current_pose.pose.orientation.x*y1 - self.current_pose.pose.orientation.y*x1 + self.current_pose.pose.orientation.z*w1
        new_w = self.current_pose.pose.orientation.w*w1 - self.current_pose.pose.orientation.x*x1 - self.current_pose.pose.orientation.y*y1 - self.current_pose.pose.orientation.z*z1

        quat = [
            new_x,
            new_y,
            new_z,
            new_w
            ]

        self.compute_waypoints(self.x, self.y, self.z, quat[0], quat[1], quat[2], quat[3])


    def broadcast_transforms_timer_callback(self):
        
        if self.transforms_to_broadcast:
            dt = 0.01  # matches your timer period
        else: return
        # Advance the orbit angle
        self._orbit_angle = (self._orbit_angle + self._orbit_speed * dt) % (2 * math.pi)

        # Recompute the pose_01 XY position if base is initialized
        if self._pose_01_base_x is not None:
            ox = self._pose_01_base_x + self._orbit_radius * math.cos(self._orbit_angle) + 0.1
            oy = self._pose_01_base_y + self._orbit_radius * math.sin(self._orbit_angle) 

            # Find and update the initial_pose transform in-place
            for tf in self.transforms_to_broadcast:
                if tf.child_frame_id == 'pose_01':
                    tf.header.stamp = self.get_clock().now().to_msg()
                    tf.transform.translation.x = ox
                    tf.transform.translation.y = oy

                    # Broadcast all transforms first, then look up target_pose in world frame
                    break

            # Broadcast first so TF tree is current
            for transform in self.transforms_to_broadcast:
                transform.header.stamp = self.get_clock().now().to_msg()
                self.tf_broadcaster.sendTransform(transform)

            # Now look up target_pose in base_link frame and publish
            if self._target_pose_offset is not None: 
                try:
                    self.target_pose = self.get_parameter('target_pose').get_parameter_value().string_value
                    t = self.tf_buffer.lookup_transform(
                        'base_link', self.target_pose,
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.01)
                    )
                    msg = PoseStamped()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'base_link'
                    msg.pose.position.x = t.transform.translation.x
                    msg.pose.position.y = t.transform.translation.y
                    msg.pose.position.z = t.transform.translation.z
                    msg.pose.orientation.x = t.transform.rotation.x
                    msg.pose.orientation.y = t.transform.rotation.y
                    msg.pose.orientation.z = t.transform.rotation.z
                    msg.pose.orientation.w = t.transform.rotation.w
                    self.target_pose_publisher.publish(msg)
                except Exception as e:
                    self.get_logger().warn(f'target_pose not in TF yet: {e}', throttle_duration_sec=2.0)

        for transform in self.transforms_to_broadcast:
            transform.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(transform)

        # if self.transforms_to_broadcast:
        #     for transform in self.transforms_to_broadcast:
        #         transform.header.stamp = self.get_clock().now().to_msg()
        #         self.tf_broadcaster.sendTransform(transform)
        # else:
        #     print('transform_not_found')
    
    
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

        self._pose_01_base_x = x
        self._pose_01_base_y = y
        self._pose_01_base_z = z 
        
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

        # T_base_pose_01: base_link → pose_01
        T_base_pose_01 = quaternion_matrix([o_x, o_y, o_z, o_w])
        T_base_pose_01[0, 3] = x
        T_base_pose_01[1, 3] = y
        T_base_pose_01[2, 3] = z

        # T_base_tool0: base_link → tool0 (from TF tree)
        t = self.tf_buffer.lookup_transform(
            'base_link', 'tool0',
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=5.0)
        )
        T_base_tool0 = quaternion_matrix([
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w,
        ])
        T_base_tool0[0, 3] = t.transform.translation.x
        T_base_tool0[1, 3] = t.transform.translation.y
        T_base_tool0[2, 3] = t.transform.translation.z

        # T_pose_01_tool0 = inv(T_base_pose_01) @ T_base_tool0
        # i.e. pose_01 → tool0
        T_pose_01_tool0 = np.linalg.inv(T_base_pose_01) @ T_base_tool0

        # Extract translation and rotation
        q = quaternion_from_matrix(T_pose_01_tool0)  # [x, y, z, w]

        self._target_pose_offset = (
            T_pose_01_tool0[0, 3],
            T_pose_01_tool0[1, 3],
            T_pose_01_tool0[2, 3],
            q[0], q[1], q[2], q[3]
        )
        self.get_logger().info(f'target_pose offset computed: {self._target_pose_offset}')


        # Broadcast target_pose as child of pose_01
        self.pose_msg_03 = PoseStamped()
        self.pose_msg_03.header.stamp = self.get_clock().now().to_msg()
        self.pose_msg_03.header.frame_id = 'pose_01'
        self.pose_msg_03.pose.position.x    = self._target_pose_offset[0]
        self.pose_msg_03.pose.position.y    = self._target_pose_offset[1]
        self.pose_msg_03.pose.position.z    = self._target_pose_offset[2]
        self.pose_msg_03.pose.orientation.x = 0.0
        self.pose_msg_03.pose.orientation.y = 0.0
        self.pose_msg_03.pose.orientation.z = 0.0
        self.pose_msg_03.pose.orientation.w = 1.0

        self.boradcast_to_tf(self.pose_msg_03, 'motion_tracking_pose')
        self.boradcast_to_tf(self.pose_msg_02, 'pose_02')
        self.boradcast_to_tf(self.pose_msg_01, 'pose_01')
        self.boradcast_to_tf(self.pose_msg_00, 'pose_00')
        
        
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


#################################################
# Main
#################################################
def main(args=None):
    rclpy.init(args=args)
    online_target_pose = OnlineTargetPose()
    
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(online_target_pose)

    # Spin until base_link and tool0 is broadcasting to this node
    while rclpy.ok():
        executor.spin_once()
        try:
            online_target_pose.tf_buffer.lookup_transform(
                'base_link', 'tool0',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.0)  # non-blocking check
            )
            online_target_pose.get_logger().info('TF ready, computing waypoints.')
            break
        except Exception:
            pass  

    if online_target_pose.use_dummy_waypoint:
        online_target_pose.compute_waypoints(
            online_target_pose.x,
            online_target_pose.y,
            online_target_pose.z,
            online_target_pose.quat[0],
            online_target_pose.quat[1],
            online_target_pose.quat[2],
            online_target_pose.quat[3]
        )
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        online_target_pose.get_logger().error(f'Exception in executor: {e}')
    finally:
        # Proper cleanup order
        executor.remove_node(online_target_pose)
        online_target_pose.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()