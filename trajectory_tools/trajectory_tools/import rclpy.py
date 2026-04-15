import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.action import ActionClient, ActionServer
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry, Path as Path_msgs
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav2_msgs.action import NavigateToPose, AssistedTeleop
from trajectory_tools_interfaces.action import FollowTrajectory
from trajectory_tools_interfaces.srv import TrajectorySmoother

import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped
from builtin_interfaces.msg import Time

from pathlib import Path
from collections import deque
import json
import numpy as np
from math import sqrt

# ========================== CLASS ==========================

class TrajectoryRetracing(Node):

    # ====================== INIT ======================

    def __init__(self):
        super().__init__('trajectory_retracing')
        self.get_logger().info('Node started')

        # -------- Parameters / State --------
        self.log_dir = Path(__file__).resolve().parent.parent.parent.parent / "src" / "trajectory_logger" / "log"
        self.fjson_name = self.log_dir / "history.json"
        self.filename = None

        self.poses = None
        self.current_position = (0.0, 0.0, 0.0, 0.0, np.array([0.0, 0.0, 0.0, 1.0]))

        self.current_index = 0
        self.dist_min = 0
        self.dist_to_validate_goal = 0.2
        self.K = 0.3
        self.Kd = 1.2
        self.v_min = 0.01
        self.v_max = 0.3
        self.lookahead_distance = 0.3

        self.map_name = None
        self.robot_state = ""
        self.first_pos_to_reach = None

        # -------- Services Clients --------
        self.client_getting_map = self.create_client(Trigger, '/get_map_name')
        self.client_pause_recording = self.create_client(Trigger, '/pause_recording')
        self.client_play_recording = self.create_client(Trigger, '/play_recording')
        self.client_get_log_dir = self.create_client(Trigger, '/get_log_dir')
        self.client_get_trajectory_smoothed = self.create_client(TrajectorySmoother, '/get_trajectory_smoothed')

        # -------- Publishers --------
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel_teleop', 10)

        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        self.smoothed_path_publisher = self.create_publisher(Path_msgs, '/smoothed_path', latched_qos)
        self.planned_path_publisher = self.create_publisher(Path_msgs, '/planned_path', latched_qos)

        # -------- TF --------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # -------- Actions --------
        self.navigate_to_pose = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.assisted_teleop_client = ActionClient(self, AssistedTeleop, '/assisted_teleop')
        self.action_server = ActionServer(self, FollowTrajectory, 'follow_trajectory', self.execute_callback)

        # -------- Subscriptions / Timers --------
        self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        self.create_timer(0.1, self.following_path_callback)
        self.create_timer(1.0, self.republish_paths_timer)


    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.current_position = (x,y,z,qx,qy,qz,qw)
    
    def reading_json(self):

        if self.fjson_name.exists() and self.fjson_name.stat().st_size !=0:
            with open(self.fjson_name, 'r', encoding='utf-8') as f:
                try:
                    data = json.load(f)
                except json.JSONDecodeError:
                    self.get_logger().error("Cannot open history.json")
                    return {}
        else:
            if self.fjson_name.exists():
                if self.fjson_name.stat().st_size ==0:
                    self.get_logger().info(f"File empty: {self.fjson_name}")
                else:
                    self.get_logger().warn("history.json not founded")
                    self.get_logger().warn(f"history.json path: {self.fjson_name}")
            return {}
        
        return data


    def data_name_sorting(self, data):
        map_names = []
        for traj_name, traj_data in data.items():
            map_name = traj_data.get('map_name')
            if map_name not in map_names:
                map_names.append(map_name)
        return map_names
    
    def data_sorting(self, data, map_name):

        csv_files = []

        for traj_name, traj_data in data.items():
            map_name_on_file = traj_data.get('map_name')
            if map_name_on_file == map_name:
                csv_files.append(traj_data.get("filename"))
            
        return csv_files
    
    def finding_total_path(self, csv_file):
        data_path = deque()
        for file in csv_file:
            path_file = self.log_dir / file
            with open(path_file, 'r') as f:
                next(f) #to skip the first line
                for line in f:
                    values = line.strip().split(',')
                    timestamp = float(values[0])
                    x = float(values[1])
                    y = float(values[2])
                    z = float(values[3])
                    qx = float(values[4])
                    qy = float(values[5])
                    qz = float(values[6])
                    qw = float(values[7])
                    data_path.append((timestamp, x, y, z, qx, qy, qz, qw))

        return data_path
    
    async def trajectory_retracing(self, map_name=None):
        
        if map_name is not None:
            self.map_name = map_name
            
        else:
            self.curent_map_name = await self.get_current_map_name()
            self.map_name = self.curent_map_name
        
        data = self.reading_json()

        map_names = self.data_name_sorting(data)
        if len(map_names) == 0:
            return 

        if not self.map_name in map_names:
            self.get_logger().warn(f"The map you choose {self.curent_map_name} don't have any points, so the last map wil be replay {map_names[-1]}.\nYou can choose the map by using \"{{map_name: 'map_x'}}\"  with x the number of the map.")
            self.map_name = map_names[-1]
        
        else:
            self.get_logger().info(f"The {self.map_name} will be used")
            
        csv_files = self.data_sorting(data, self.map_name)
        
        data_path = self.finding_total_path(csv_files)

        #self.get_logger().info(f"Total points: {len(data_path)}") #debbug

        return data_path
    

    def build_poses(self, data_path):

        poses = []
        for (timestamp, x, y, z, qx, qy, qz, qw) in data_path:
            pose = PoseStamped()

            pose.header.frame_id = "map"

            pose.header.stamp = Time()
        
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z            

            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            poses.append(pose)

        return list(reversed(poses))

    async def get_current_map_name(self):

        map_name = None
        while not self.client_getting_map.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for service...")
            self.get_logger().info("/get_map_name service unavailable")
            return None

        request = Trigger.Request()
        future = self.client_getting_map.call_async(request)
        await future

        if future.result() is not None and future.result().success:
            map_name = future.result().message
            self.get_logger().info(f"Current map: {map_name}")
        return map_name
    

    async def go_to_first_pose(self):

        self.get_logger().info("go_to_first_pose started")
        x, y = self.first_pos_to_reach

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y

        goal_handle = await self.navigate_to_pose.send_goal_async(goal)
        self.get_logger().info(f"Goal accepted: {goal_handle.accepted}")

        if not goal_handle.accepted:
            self.get_logger().warn("Goal refused")
            return
        self.get_logger().info("Waiting for result...")
        result = await goal_handle.get_result_async()
        self.get_logger().info("Result received!")
        teleop_goal = AssistedTeleop.Goal()
        teleop_goal.time_allowance.sec = 0 
        self.assisted_teleop_client.send_goal_async(teleop_goal)
        self.robot_state = 'retracing'


    def finish_retracing(self):
        request_play = Trigger.Request()
        future_play = self.client_play_recording.call_async(request_play)
        future_play.add_done_callback(self.on_play_done)


    def on_play_done(self, future):
        self.get_logger().info('Ready to record again')
        result = FollowTrajectory.Result()
        result.success = True
        self._goal_handle.succeed()
        self._execute_future.set_result(result)

    def stop_robot(self):

        stop_msg = TwistStamped()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        stop_msg.header.frame_id = 'base_link'
        stop_msg.twist.linear.x = 0.0
        stop_msg.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_msg)
        self.get_logger().info("Robot stopped")

    def following_path(self, poses):

        if self.current_index >= len(poses):
            return

        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
        except Exception as e:
            self.get_logger().warn(f'Transformation failed: {e}')
            return
        
        target_pose = poses[-1]
        for i in range(self.current_index, len(poses)):
            dist = sqrt((poses[i].pose.position.x-robot_x)**2 + (poses[i].pose.position.y-robot_y)**2)
            
            if dist >= self.lookahead_distance:
                target_pose = poses[i]
                self.current_index = i
                break

        try:
            transform_inv = self.tf_buffer.lookup_transform("base_link", "map", rclpy.time.Time())
            target_local = do_transform_pose_stamped(target_pose, transform_inv)
        except Exception as e:
            return


        x_local = target_local.pose.position.x
        y_local = target_local.pose.position.y

        L_sq = x_local**2 + y_local**2 

        
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'base_link'

        
        dist_to_end = sqrt((poses[-1].pose.position.x - robot_x)**2 + 
                        (poses[-1].pose.position.y - robot_y)**2)
        
        if dist_to_end <= 0.10: 
            self.robot_state = 'finis'
            self.stop_robot()
            self.finish_retracing()
            return

        v = min(self.v_max, self.Kd * dist_to_end)
        v = max(v, self.v_min)

        if L_sq > 0.001:
            twist.twist.angular.z = v * (2.0 * y_local / L_sq)
        else:
            twist.twist.angular.z = 0.0

        twist.twist.linear.x = v
        self.cmd_vel_pub.publish(twist)


    def following_path_callback(self):

        if self.robot_state != 'retracing':
            if self.robot_state == 'finis':
             self.stop_robot()
             self.robot_state = ''
            return
        if self.poses is None:
            return
        self.following_path(self.poses)


    def publish_smoothed_trajectory(self, poses):
        path_msg = Path_msgs()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = Time()
        path_msg.poses = poses
        self.smoothed_path_publisher.publish(path_msg)


    def publish_trajectory(self, poses):
        path_msg = Path_msgs()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = Time()
        path_msg.poses = poses 
        self.planned_path_publisher.publish(path_msg)

    def republish_paths_timer(self):
        if self.poses is None:
            return

        self.publish_trajectory(self.poses)

    def get_trajectory_smoothed(self, poses):
        request = TrajectorySmoother.Request()
        request.input_data = poses
        return self.client_get_trajectory_smoothed.call_async(request)


    async def execute_callback(self, goal_handle):
        
        self.get_logger().info("Action started")

        while not self.client_pause_recording.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for pausing the record...')

        
        request_pause = Trigger.Request()
        future_pause = self.client_pause_recording.call_async(request_pause)
        
        await future_pause

        if future_pause.result() is not None and future_pause.result().success:
            self.get_logger().info('Pause of the record validated')
            self.current_index = 0
            self.robot_state = ''

            if goal_handle.request.map_name !="":
                map_name = goal_handle.request.map_name
                data_path = await self.trajectory_retracing(map_name)
            else:
                data_path = await self.trajectory_retracing()
            
            self.poses = self.build_poses(data_path)
            future_smoothed = self.get_trajectory_smoothed(self.poses)
            await future_smoothed

            if future_smoothed.result() is not None and future_smoothed.result().success:

                self.poses = future_smoothed.result().output_data
                self.get_logger().info('Trajectory smoothed successfully')

                self.publish_smoothed_trajectory(self.poses)

            else:
                self.get_logger().warn('Error while smoothing the trajectory, using raw trajectory')
            self.first_pos_to_reach = (self.poses[0].pose.position.x, self.poses[0].pose.position.y)
            
            self._goal_handle = goal_handle
            self._execute_future = rclpy.task.Future()

            await self.go_to_first_pose()

            result = await self._execute_future
            return result
        
        else:
            self.get_logger().warn("The pause wasn't accepted or received")

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryRetracing()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()