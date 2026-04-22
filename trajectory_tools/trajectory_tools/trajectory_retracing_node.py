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
from action_msgs.msg import GoalStatusArray
import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped
from builtin_interfaces.msg import Time

from pathlib import Path
from collections import deque
import json
import numpy as np
from math import sqrt, atan2
from math import pi as pi
from scipy.spatial.transform import Rotation
from tf2_ros import TransformException



class TrajectoryRetracing(Node):

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
        self.last_position = None

        self.map_name = None
        self.robot_state = ""
        self.first_pos_to_reach = None
        self.teleop_goal_handle = None
        self.my_nav_uuid = None
        self.interrupt_uuid = None
        
        self.timer_robot_stuck = None
        self._stuck_timer_running = False
        self._stuck_start_position = None        # position au début de la fenêtre de 5s
        self._stuck_threshold = 0.05 
        self.interruption_retracing = False

        # -------- Robot states --------

        self.STATE_IDLE = "IDLE"
        self.STATE_GO_TO_NAV = "GO_TO_NAV"
        self.STATE_NAV_INTERUPT = "NAV_INTERUPT"
        self.STATE_NAV_GO_BACK_ON_TRACK = "NAV_GO_BACK_ON_TRACK"
        self.STATE_NAV_AVOID_OBSTACLE = "NAV_AVOID_OBSTACLE"
        self.STATE_RETRACING = "RETRACING"
        self.STATE_STUCK = "STUCK"
        self.STATE_FINISHED = "FINISHED"

        self.robot_state = self.STATE_IDLE
        #self.get_logger().info(f"[STATE] → {self.robot_state}")

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
        self.create_subscription(GoalStatusArray, "/navigate_to_pose/_action/status", self.goal_pose_callback, 10)
        self.create_timer(0.1, self.following_path_callback)
        self.create_timer(1.0, self.republish_paths_timer)


    # ====================== DATA ======================

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
                next(f)
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

    # ====================== TRAJECTORY BUILD ======================

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.current_position = (x,y,z,qx,qy,qz,qw)

    async def trajectory_retracing(self, map_name=None):
        if map_name is not None:
            self.map_name = map_name
        else:
            self.current_map_name = await self.get_current_map_name()
            self.map_name = self.current_map_name

        data = self.reading_json()
        map_names = self.data_name_sorting(data)
        if len(map_names) == 0:
            self.get_logger().error('No map recorded, cannot retrace')
            return deque()

        if not self.map_name in map_names:
            self.get_logger().warn(f"The map you choose {self.current_map_name} don't have any points, so the last map wil be replay {map_names[-1]}.\nYou can choose the map by using \"{{map_name: 'map_x'}}\"  with x the number of the map.")
            self.map_name = map_names[-1]
        else:
            self.get_logger().info(f"The {self.map_name} will be used")

        csv_files = self.data_sorting(data, self.map_name)
        if not csv_files:
            self.get_logger().error(f"No csv file found around: {self.map_name}")
            return deque()
        
        data_path = self.finding_total_path(csv_files)
        return data_path

    def build_poses(self, data_path):

        if not data_path:
            return []
        
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
    
        # ====================== MAP / NAVIGATION HELPERS ======================
    def cancel_stuck_timer(self):
        if self._stuck_timer_running:
            self.timer_robot_stuck.cancel()
            self.timer_robot_stuck = None
            self._stuck_timer_running = False

    def _cancel_stuck_timer_and_resume(self):
        if self._stuck_timer_running:
            self.cancel_stuck_timer()
            if self.robot_state == self.STATE_STUCK:
                self.get_logger().info('Robot moving again, resuming retracing')
                self.robot_state = self.STATE_RETRACING

    def _is_mooving(self, robot_x, robot_y):
        if self.last_position is None:
            return True 
                    
        dist = sqrt((robot_x - self.last_position[0])**2 +
                    (robot_y - self.last_position[1])**2)
        if dist < 0.01:
            if not self._stuck_timer_running:
                self.timer_robot_stuck = self.create_timer(5.0, self.timer_robot_stuck_callback)
                self._stuck_timer_running = True
            return False
        
        else:
            self._cancel_stuck_timer_and_resume()
            return True
    
    def _aboart_action(self):
        self.stop_robot()
        self.stop_assisted_teleop()
        self.get_logger().info('The retracing action is stop, re-launch the action server to join back the path and continue the retracing.')
        self.robot_state = self.STATE_IDLE
        result = FollowTrajectory.Result()
        result.success = False
        self.interruption_retracing = True

        if hasattr(self, 'goal_handle') and self._goal_handle is not None:
            self._goal_handle.abort()
            self._goal_handle = None
        
        if hasattr(self, '_execute_future') and self._execute_future is not None:
            if not self._execute_future.done():
                self._execute_future.set_result(None)
            self._execute_future = None

    def timer_robot_stuck_callback(self):
        if self.robot_state == self.STATE_STUCK:
            return
        self.robot_state = self.STATE_STUCK
        self.cancel_stuck_timer()
        self.stop_robot()
        self.stop_assisted_teleop()
        self.get_logger().error('The robot is stuck, use the Nav 2 goal tool to avoid the obstacle.')
        self._stuck_start_position = self.last_position
        self.avoid_obstacle(self._stuck_start_position)

            

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
    
    def _run_go_to_closest_point_once(self):
        self._retry_timer.cancel()
        self.destroy_timer(self._retry_timer)
        self.go_to_closest_point()


    def goal_pose_callback(self, msg):
        INTERNAL_STATES = {
                            self.STATE_NAV_AVOID_OBSTACLE,
                            self.STATE_NAV_GO_BACK_ON_TRACK,
                            self.STATE_GO_TO_NAV,
                            }
        if self.robot_state == self.STATE_RETRACING:
            for status in msg.status_list:
                if status.status==2:
                    uuid = tuple(status.goal_info.goal_id.uuid)
                    if uuid != self.my_nav_uuid:
                        self.interrupt_uuid = uuid
                        self.get_logger().info("Goal Nav2 detected")
                        self.stop_robot()
                        self.cancel_stuck_timer()
                        self.robot_state = self.STATE_NAV_INTERUPT
                        #self.get_logger().info(f"[STATE] → {self.robot_state}")
                        self.stop_assisted_teleop()
        
        if self.robot_state == self.STATE_NAV_INTERUPT:
            for status in msg.status_list:

                if status.status==4:
                
                    uuid = tuple(status.goal_info.goal_id.uuid)
                    if uuid == self.interrupt_uuid:

                        self.interrupt_uuid = None
                        self.get_logger().info('Intermediate point reached, back to the retracing')
                        #ici créer une pause, on arrête l'action serveur en enregistrant une variable d'état et à son démarrage, je vais vers la fin
                        self._aboart_action()
    
    def compute_yaw_for_retracing(self, pos=1):
        if self.poses is None or len(self.poses) < 2:
            return 0.0

        target_distance = 0.2  # 20 cm
        acc_dist = 0.0

        dx_total = 0.0
        dy_total = 0.0

        for i in range(pos, len(self.poses)):
            p_prev = self.poses[i - 1].pose.position
            p_cur = self.poses[i].pose.position

            dx = p_cur.x - p_prev.x
            dy = p_cur.y - p_prev.y

            step_dist = sqrt(dx * dx + dy * dy)

            if acc_dist + step_dist > target_distance:
                ratio = (target_distance - acc_dist) / step_dist if step_dist > 0 else 0.0
                dx_total += dx * ratio
                dy_total += dy * ratio
                break

            dx_total += dx
            dy_total += dy
            acc_dist += step_dist

        return atan2(dy_total, dx_total)
    
    def yaw_to_quaternion(self, yaw):
        qz = np.sin(yaw / 2.0)
        qw = np.cos(yaw / 2.0)
        return qz, qw

    def closest_point_on_the_path(self):

        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
        except Exception as e:
            self.get_logger().warn(f'Transformation failed (closest_point_on_the_path): {e}')
            return
        
        dist_min = float('inf')

        target_index = self.current_index

        for i in range(0, len(self.poses)):
            dx = self.poses[i].pose.position.x - robot_x
            dy = self.poses[i].pose.position.y - robot_y
            i_dist = dx * dx + dy * dy

            if i_dist < dist_min:
                dist_min = i_dist
                target_index = i

        self.current_index = target_index
        return self.poses[target_index]

    def _on_teleop_sent(self, future):
        self.teleop_goal_handle = future.result()

    def _on_goal_finished(self, future):
        self.get_logger().info("Back to track finished")

        teleop_goal = AssistedTeleop.Goal()
        teleop_goal.time_allowance.sec = 0

        send_future = self.assisted_teleop_client.send_goal_async(teleop_goal)
        send_future.add_done_callback(self._on_teleop_sent)
 
        self.robot_state = self.STATE_RETRACING
        #self.get_logger().info(f"[STATE] → {self.robot_state}")
        self.get_logger().info('Restart retracing')

    def _on_avoid_obstacle_goal_finished(self, future):
        self._retry_timer = self.create_timer(0.0, self._run_go_to_closest_point_once)

    def _on_goal_sent(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("Goal refused")
            return

        goal_handle.get_result_async().add_done_callback(self._on_goal_finished)

    def _on_avoid_obstacle_goal_sent(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal refused")
            return
        goal_handle.get_result_async().add_done_callback(self._on_avoid_obstacle_goal_finished)


    def avoid_obstacle(self, position_to_reach):
        if self.robot_state != self.STATE_STUCK:
            return
        
        self.cancel_stuck_timer()
        target_distance = 0.2 #20cm
        index_to_reach = self.current_index
        
        if self.robot_state == self.STATE_STUCK:
            self.get_logger().info('Avoiding obstacle')
            
            for i in range(self.current_index, len(self.poses)):
                p_cur = self.poses[self.current_index].pose.position
                p_to_reach = self.poses[i].pose.position

                dx = p_cur.x - p_to_reach.x
                dy = p_cur.y - p_to_reach.y

                step_dist = sqrt(dx * dx + dy * dy)
                
                if step_dist > target_distance:
                    index_to_reach = i
                    break
            
            target_pose = self.poses[index_to_reach]
            self.robot_state = self.STATE_NAV_AVOID_OBSTACLE

            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = "map"
            goal.pose.pose.position.x = target_pose.pose.position.x
            goal.pose.pose.position.y = target_pose.pose.position.y

            yaw = self.compute_yaw_for_retracing()# A tenter avec l'argument self.current_index
            qz, qw = self.yaw_to_quaternion(yaw)

            goal.pose.pose.orientation.z = qz
            goal.pose.pose.orientation.w = qw

            future = self.navigate_to_pose.send_goal_async(goal)
            future.add_done_callback(self._on_avoid_obstacle_goal_sent)


    def go_to_closest_point(self):
        self.get_logger().info('Go to the closest point on the track')
        self.robot_state = self.STATE_NAV_GO_BACK_ON_TRACK
        #self.get_logger().info(f"[STATE] → {self.robot_state}")
        target_pose = self.closest_point_on_the_path()

        x = target_pose.pose.position.x
        y = target_pose.pose.position.y

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y

        yaw = self.compute_yaw_for_retracing(self.current_index)
        qz, qw = self.yaw_to_quaternion(yaw)

        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        future = self.navigate_to_pose.send_goal_async(goal)
        future.add_done_callback(self._on_goal_sent)
            

    async def go_to_first_pose(self):
        self.get_logger().info("go_to_first_pose: Started")
        self.robot_state = self.STATE_GO_TO_NAV
        #self.get_logger().info(f"[STATE] → {self.robot_state}")

        x, y = self.first_pos_to_reach
        

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y

        yaw = self.compute_yaw_for_retracing()
        qz, qw = self.yaw_to_quaternion(yaw)

        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw


        goal_handle = await self.navigate_to_pose.send_goal_async(goal)
        # self.get_logger().info(f"Goal accepted: {goal_handle.accepted}")
        self.my_nav_uuid = tuple(goal_handle.goal_id.uuid) #To keep a track of my uuid

        if not goal_handle.accepted:
            self.get_logger().warn("Goal refused")
            return

        # self.get_logger().info("go_to_first_pose: Waiting for result...")
        result = await goal_handle.get_result_async()
        # self.get_logger().info("go_to_first_pose: Result received!")

        teleop_goal = AssistedTeleop.Goal()
        teleop_goal.time_allowance.sec = 0
        send_future = self.assisted_teleop_client.send_goal_async(teleop_goal)
        self.teleop_goal_handle = await send_future

        self.robot_state = self.STATE_RETRACING
        #self.get_logger().info(f"[STATE] → {self.robot_state}")

    def finish_retracing(self):
        self.robot_state = self.STATE_IDLE
        self.interruption_retracing = False
        #self.get_logger().info(f"[STATE] → {self.robot_state}")
        self.stop_robot()
        self.stop_assisted_teleop()
        request_play = Trigger.Request()
        future_play = self.client_play_recording.call_async(request_play)
        future_play.add_done_callback(self.on_play_done)
        

    def on_play_done(self, future):
        self.get_logger().info('Ready to record again')
        result = FollowTrajectory.Result()
        result.success = True
        self._goal_handle.succeed()
        self._execute_future.set_result(result)

    # ====================== ROBOT CONTROL ======================

    def stop_assisted_teleop(self):
        if self.teleop_goal_handle is None:
            return

        self.get_logger().info("Cancel AssistedTeleop")

        future = self.teleop_goal_handle.cancel_goal_async()
        future.add_done_callback(self._on_teleop_cancelled)

    def _on_teleop_cancelled(self, future):
        self.get_logger().info('Teleop cancelled')
        self.teleop_goal_handle = None

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
        
        if self.robot_state == self.STATE_NAV_INTERUPT:
            return
        
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
        except Exception as e:
            self.get_logger().warn(f'Transformation failed (following path): {e}')
            return

        if not self._is_mooving(robot_x, robot_y):
           return
        
        self.last_position = (robot_x, robot_y)

        target_pose = poses[-1]
        for i in range(self.current_index, len(poses)):
            dist = sqrt((poses[i].pose.position.x - robot_x) ** 2 +
                        (poses[i].pose.position.y - robot_y) ** 2)

            if dist >= self.lookahead_distance:
                target_pose = poses[i]
                self.current_index = i
                break

        try:
            transform_inv = self.tf_buffer.lookup_transform("base_link", "map", rclpy.time.Time())
            target_local = do_transform_pose_stamped(target_pose, transform_inv)
        except Exception:
            return

        x_local = target_local.pose.position.x
        y_local = target_local.pose.position.y
        L_sq = x_local ** 2 + y_local ** 2

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'base_link'

        dist_to_end = sqrt((poses[-1].pose.position.x - robot_x)**2 + (poses[-1].pose.position.y - robot_y)**2)

        if dist_to_end <= 0.10:
            self.robot_state = self.STATE_FINISHED
            #self.get_logger().info(f"[STATE] → {self.robot_state}")
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

        if self.robot_state != self.STATE_RETRACING:
            if self.robot_state == self.STATE_FINISHED:
                self.stop_robot()
                self.robot_state = self.STATE_IDLE
                #self.get_logger().info(f"[STATE] → {self.robot_state}")
            return

        if self.poses is None:
            return

        

        self.following_path(self.poses)


    # ====================== PATH PUBLISHING ======================

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
        self.publish_smoothed_trajectory(self.poses)
        
    # ====================== SMOOTHING SERVICE ======================

    def get_trajectory_smoothed(self, poses):
        request = TrajectorySmoother.Request()
        request.input_data = poses
        return self.client_get_trajectory_smoothed.call_async(request)

    # ====================== ACTION SERVER ======================

    async def execute_callback(self, goal_handle):

        self.get_logger().info("Action started")

        while not self.client_pause_recording.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for pausing the record...')

        request_pause = Trigger.Request()
        future_pause = self.client_pause_recording.call_async(request_pause)
        await future_pause

        if self.interruption_retracing:
            self.get_logger().info('Resume retracing from closest point on the path')
            self.interruption_retracing = False

            self._goal_handle = goal_handle
            self._execute_future = rclpy.task.Future()
        
            self.go_to_closest_point()

            result = await self._execute_future
            return result


        if future_pause.result() is not None and future_pause.result().success:

            self.get_logger().info('Pause of the record validated')
            self.current_index = 0
            self.robot_state = self.STATE_IDLE
            #self.get_logger().info(f"[STATE] → {self.robot_state}")

            if goal_handle.request.map_name != "":
                map_name = goal_handle.request.map_name
                data_path = await self.trajectory_retracing(map_name)
                if not data_path:
                    self.get_logger().error("No trajectory data found. Aborting action.")
                    goal_handle.abort()
                    result = FollowTrajectory.Result()
                    result.success = False
                    return result
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

            self.first_pos_to_reach = (
                self.poses[0].pose.position.x,
                self.poses[0].pose.position.y
            )

            self._goal_handle = goal_handle
            self._execute_future = rclpy.task.Future()

            await self.go_to_first_pose()
            self.get_logger().info('go_to_first_pose: Succeded')

            result = await self._execute_future

            if result is None:
                r = FollowTrajectory.Result()
                r.success = False
                return r
            
        else:
            self.get_logger().warn("The pause wasn't accepted or received")
            self.stop_assisted_teleop()



    def destroy_node(self):

        self.get_logger().info("Shutting down node...")
        self.stop_assisted_teleop()
        super().destroy_node()


# ========================== MAIN ==========================

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