"""
Version 3: This version works with the action server 'trajectory_tools'. Every decision has been made by the action server.
It decide when the recording of the trajectories will start and end thanks to services. It also decide about the logic. (recording a path)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from collections import deque
import csv
from pathlib import Path
from math import sqrt
from std_srvs.srv import Trigger
from trajectory_tools_interfaces.srv import StopRecording
import json
from nav_msgs.msg import Path as Path_msgs
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped



class TrajectoryLogger(Node):
    def __init__(self):
        super().__init__('trajectory_logger')
        self.get_logger().info('Node started')
        

        ######## PARAMETERS ########

        self.record_duration = 1 #in seconds
        self.record_frequency = 48 #frequency of /odom (48 for gazebo)
        self.writing_frequency = 0.5 #in seconds
        self.precision = 12 # 6 = 10e-6

        ######## TOLERANCES ########

        self.epsilon_speed = 0.02 # in (m/s). Minimum speed at which the robot is considered stopped 
        self.epsilon_rotation = 0.02 # in rad/s. Minimum rotation speed at which the robot is considered as not mooving 
        ###########################

        self.first_position_path = None
        self.last_position_path = None

        self.timestamp_previous = None
        self.total_points = 0

    

        self.first_angle_of_rotation = None
        self.last_angle_of_rotation = None

        self.last_odom_timestamp = None

        self.filename = None
        self.map_name = "map_1"
        self.last_recorded_position = None
        self.recording_enabled = None
        self.current_goal_id = None

        self.json_recorded = False

        self.timer_write = self.create_timer(
            self.writing_frequency,
            self.timer_write_callback
        )
    

        self.create_subscription(
            Odometry,
            "odom", #odometry/filtered
            self.odom_callback,
            10
        )

        self.path_publisher_logger = self.create_publisher(Path_msgs, '/planned_path_logger', 10)
        self.path_msg = Path_msgs()
        self.path_msg.header.frame_id = "map"
        
        self.length_buffer = int(self.record_duration * self.record_frequency*2)
        self.buffer=deque(maxlen=self.length_buffer)


        self.log_dir = Path(__file__).resolve().parent.parent / "log"
        self.log_dir.mkdir(parents=True, exist_ok=True) #Create the log file if it wasn't created 
        self.file_index = None
        self.fjson_name = self.log_dir / "history.json"
        self.map_identification()


        self.create_service(
            Trigger,
            '/get_log_dir',
            self.get_log_dir_callback
            )

        self.start_service = self.create_service(
            Trigger,
            'start_recording',
            self.start_recording_callback
        )

        self.stop_service = self.create_service(
            StopRecording,
            'stop_recording',
            self.stop_recording_callback
        )

        self.get_map_name = self.create_service(
            Trigger,
            '/get_map_name',
            self.get_map_name_callback
        )
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


    def get_log_dir_callback(self, request, response):
        response.success = True
        response.message = str(self.log_dir)
        return response

        

    def get_map_name_callback(self, request, response):
        self.get_logger().info(f"Map name asked : {self.map_name}")
        response.success = True
        response.message = self.map_name

        return response

    def map_identification(self):
        if not self.fjson_name.exists():
            return
        with open(self.fjson_name, 'r', encoding="utf-8") as f:
                try:
                    data = json.load(f)
                
                except json.JSONDecodeError:
                    data = {}

        if len(data) == 0:
            return
        last_record = list(data.values())[-1]
        last_map = int(last_record["map_name"].split("_")[1])
        self.map_name = f"map_{last_map+1}"

    
    def start_recording_callback(self, request, response):
        self.get_logger().info("Recording started !")
        self.recording_enabled = True   
        #self.fjson_name = self.log_dir / f"history.json"  
        self.new_file()
        self.first_position_path = None
        response.success = True
        response.message = 'Recording started !'
  
        return response
    
    def stop_recording_callback(self, request, response):
        self.get_logger().info("Recording stopped !")
        self.recording_enabled = False
        self.timer_write_callback()
        response.success = True

        if hasattr(self, "filename") and self.filename is not None and not self.json_recorded:
            self.json_record()
            

        if hasattr(self, "file") and not self.file.closed:
            response.filename = self.filename.name
            response.file_location = str(self.log_dir) 
            response.total_points = self.total_points
            
        return response

    def json_record(self):
        if self.last_recorded_position is None:
            return
        
        x = self.last_recorded_position[1]
        y = self.last_recorded_position[2]

        record = {
            "map_name": self.map_name,
            "filename": str(self.filename.name) if self.filename else "N/A",
            "first_point": self.first_position_path,
            "last_point": (x, y)
        }

        if self.fjson_name.exists():
            with open(self.fjson_name, 'r', encoding="utf-8") as f:
                try:
                    data = json.load(f)
                except json.JSONDecodeError:
                    data = {}
        else:
            data = {}

        if self.filename:
            data[str(self.filename.name)] = record

        with open(self.fjson_name, 'w', encoding="utf-8") as f:
            json.dump(data, f, indent=4)
        
        self.json_recorded = True
        self.get_logger().info('Writing history...')

    def new_file(self):
        if self.file_index is None:
            self.file_index = max(
                (int(f.stem.split("_")[2]) for f in self.log_dir.glob("trajectory_logger_*.csv")),
                default=0)
        self.file_index +=1
        self.filename = self.log_dir / f"trajectory_logger_{self.file_index}.csv"
        self.get_logger().info(f"Log file: {self.filename}")
        self.file=open(self.filename, 'w', newline='') #Opening the file
        self.writer = csv.writer(self.file)
        self.writer.writerow(["timestamp", "x", "y", "z", "qx", "qy", "qz", "qw"]) #Addind header to record_location_values
        self.total_points = 0
        self.json_recorded = False

        if self.last_recorded_position is not None:
            self.writer.writerow(self.last_recorded_position)
            



    def should_record(self, vx, rz):
        if vx > self.epsilon_speed or abs(rz) > self.epsilon_rotation:
            return True
        else:
            return False
 
    

    def odom_callback(self, msg):

        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
       
        
        vx = msg.twist.twist.linear.x
        rz = msg.twist.twist.angular.z 

        if self.recording_enabled:
            self.last_odom_timestamp = timestamp

            self.timestamp_previous = timestamp

            x = round(msg.pose.pose.position.x, self.precision)
            y = round(msg.pose.pose.position.y, self.precision)
            z = round(msg.pose.pose.position.z, self.precision)
            qx = round(msg.pose.pose.orientation.x, self.precision)
            qy = round(msg.pose.pose.orientation.y, self.precision)
            qz = round(msg.pose.pose.orientation.z, self.precision)
            qw = round(msg.pose.pose.orientation.w, self.precision)

            pose_in_odom = PoseStamped()
            pose_in_odom.header.stamp = self.get_clock().now().to_msg()
            pose_in_odom.header.frame_id = "odom" 
            pose_in_odom.pose.position.x = x
            pose_in_odom.pose.position.y = y
            pose_in_odom.pose.position.z = z
            pose_in_odom.pose.orientation.x = qx
            pose_in_odom.pose.orientation.y = qy
            pose_in_odom.pose.orientation.z = qz
            pose_in_odom.pose.orientation.w = qw

            try:
                
                transform = self.tf_buffer.lookup_transform("map", "odom", rclpy.time.Time())
                pose_in_map = do_transform_pose_stamped(pose_in_odom, transform)
                
                x_after_tf = round(pose_in_map.pose.position.x, self.precision)
                y_after_tf = round(pose_in_map.pose.position.y, self.precision)
                z_after_tf = round(pose_in_map.pose.position.z, self.precision)
                qx_after_tf = round(pose_in_map.pose.orientation.x, self.precision)
                qy_after_tf = round(pose_in_map.pose.orientation.y, self.precision)
                qz_after_tf = round(pose_in_map.pose.orientation.z, self.precision)
                qw_after_tf = round(pose_in_map.pose.orientation.w, self.precision)

                if self.first_position_path is None:
                    self.first_position_path = (x_after_tf, y_after_tf)

            except Exception as e:
                self.get_logger().warn(f'Transformation failed: {e}')
                return
                          
            
            self.buffer.append((timestamp, x_after_tf, y_after_tf, z_after_tf, qx_after_tf, qy_after_tf, qz_after_tf, qw_after_tf))
            self.last_recorded_position = (timestamp,
                                           x_after_tf,
                                           y_after_tf,
                                           z_after_tf,
                                           qx_after_tf,
                                           qy_after_tf,
                                           qz_after_tf,
                                           qw_after_tf)
            self.total_points += 1

            self.path_msg.poses.append(pose_in_map)
     
            # Publier le Path sur le topic
            self.path_msg.header.stamp = self.get_clock().now().to_msg()
            self.path_publisher_logger.publish(self.path_msg)
                

    def timer_write_callback(self):
        #self.get_logger().info(f"{len(self.buffer)}")
        if not hasattr(self, "file") or self.file is None or self.file.closed:
            return

        
        if len(self.buffer) == 0:
            return
      
        if len(self.buffer) >0:
            self.writer.writerows(self.buffer)
            self.file.flush() #To force the disk to write
            self.buffer.clear()
            

        
    def destroy_node(self):

        self.get_logger().info("Shutting down node...")

        if hasattr(self, "writer") and len(self.buffer) > 0:
            self.writer.writerows(self.buffer)
            self.file.flush()

        if hasattr(self, "file"):
            self.file.close()


        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)
    node = TrajectoryLogger()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received")

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()