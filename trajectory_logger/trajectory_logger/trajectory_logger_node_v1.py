"""
Version 1: Records everything without exception.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from collections import deque
import csv
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


class TrajectoryLogger(Node):
    def __init__(self):
        super().__init__('trajectory_logger')
        self.get_logger().info('Node started')

        ######## PARAMETERS ########

        self.record_duration = 1 #in seconds
        self.record_frequency = 48 #frequency of /odom (48 for gazebo)
        self.writing_frequency = 0.5 #in seconds
        ###########################

        self.timestamp_previous = None
        self.msg_count = 0

        self.timer_info = self.create_timer(
            1,
            self.display_infos
        )

        self.timer_write = self.create_timer(
            self.writing_frequency,
            self.timer_write_callback
        )

        self.create_subscription(
            Odometry,
            "odom",
            self.odom_callback,
            10
        )
        
        self.length_buffer = int(self.record_duration * self.record_frequency*2)
        self.buffer=deque(maxlen=self.length_buffer)


        workspace = Path(__file__).resolve().parents[6]
        log_dir = workspace / "src" / "trajectory_logger" / "log"
        log_dir.mkdir(parents=True, exist_ok=True) #Create the log file if it wasn't created 
        now=datetime.now()
        filename = log_dir / f"trajectory_logger_{now.strftime('%Y%m%d_%H%M%S')}.csv"

        self.file=open(filename, 'a', newline='') #Opening the file
        self.writer = csv.writer(self.file)
        self.writer.writerow(["timestamp","x","y","z","qx","qy","qz","qw"]) #Addind header to record_location_values

        self.get_logger().info(f"Log directory: {log_dir}")
        self.get_logger().info(f"Log file: {filename}")

    def odom_callback(self, msg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        if self.timestamp_previous is not None:
            dt = timestamp - self.timestamp_previous
            
            if dt <= 0:
                return

            expected_dt = 1 / self.record_frequency #To calculate if all the messages arrived

            if dt >expected_dt * 1.5:
                self.get_logger().warn("Possible message loss")


            
            
        self.timestamp_previous = timestamp

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.buffer.append((timestamp, x, y, z, qx, qy, qz, qw))
        self.msg_count += 1

    def timer_write_callback(self):
        #self.get_logger().info(f"{len(self.buffer)}")
        if len(self.buffer) == 0:
            return

        self.writer.writerows(self.buffer)
        self.file.flush() #To force the disk to write

        self.buffer.clear()

    def display_infos(self):
        self.get_logger().info(f"Frequency : {self.msg_count} Hz")
        self.msg_count = 0

    def destroy_node(self):

        self.get_logger().info("Shutting down node...")

        if len(self.buffer) > 0:
            self.writer.writerows(self.buffer)
            self.file.flush()

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