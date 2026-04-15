import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import ActionClient
from nav2_msgs.action import FollowPath
from std_srvs.srv import Trigger
import json
from pathlib import Path
from collections import deque
from trajectory_tools_interfaces.action import FollowTrajectory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as Path_msgs



class TrajectoryRetracing(Node):
    def __init__(self):
        super().__init__('trajectory_retracing')
        self.get_logger().info('Node started')



        self.client_getting_map = self.create_client(Trigger, '/get_map_name')
        self.client_pause_recording = self.create_client(Trigger, '/pause_recording')
        self.client_play_recording = self.create_client(Trigger, '/play_recording')
        self.client_get_log_dir = self.create_client(Trigger, '/get_log_dir')
        self.log_dir = Path(__file__).resolve().parent.parent.parent.parent / "src" / "trajectory_logger" / "log"
        self.fjson_name = self.log_dir / "history.json"
        self.filename = None
        
        #self.path_publisher = self.create_publisher(Path_msgs, '/planned_path', 10)

        self.map_name = None

        self.action_client = ActionClient(
            self,
            FollowPath,
            '/follow_path'
        )

        self.action_server = ActionServer(
            self,
            FollowTrajectory,
            'follow_trajectory',
            self.execute_callback            
        )

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

        self.get_logger().info(f"Total points: {len(data_path)}") #debbug

        return data_path
    

    def build_poses(self, data_path):
        
        poses = []
        for (timestamp, x, y, z, qx, qy, qz, qw) in data_path:
            pose = PoseStamped()

            pose.header.frame_id = "map"

            pose.header.stamp = self.get_clock().now().to_msg()
        
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z            

            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            poses.append(pose)
        
        return list(reversed(poses))


    async def send_to_nav2(self, poses):
        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Waiting for /follow_path...")

        goal = FollowPath.Goal()

        path = Path_msgs()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = poses
        goal.path = path
        future = self.action_client.send_goal_async(goal)

        await future

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2")
            return
            
        result_future = goal_handle.get_result_async()
        await result_future
        self.get_logger().info("Trajectory completed !")
        
    def publish_trajectory(self, poses):
        path_msg = Path_msgs()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.poses = poses 
        self.path_publisher.publish(path_msg)


    async def execute_callback(self, goal_handle):
        self.get_logger().info("Action started")

        while not self.client_pause_recording.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for pausing the record...')
            
        
        request_pause = Trigger.Request()
        future_pause = self.client_pause_recording.call_async(request_pause)
        
        await future_pause

        if future_pause.result() is not None and future_pause.result().success:
            self.get_logger().info('Pause of the record validated')

            if goal_handle.request.map_name !="":
                map_name = goal_handle.request.map_name
                data_path = await self.trajectory_retracing(map_name)
            else:
                data_path = await self.trajectory_retracing()
            
            poses = self.build_poses(data_path)

            #self.publish_trajectory(poses)

            await self.send_to_nav2(poses)
            
            while not self.client_play_recording.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for recording again...')
                
            
            request_play= Trigger.Request()
            future_play = self.client_play_recording.call_async(request_play)
            await future_play

            if future_play.result() is not None and future_play.result().success:
                self.get_logger().info('Ready to record again')
                
            goal_handle.succeed()
            result = FollowTrajectory.Result()
            result.success = True
            
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