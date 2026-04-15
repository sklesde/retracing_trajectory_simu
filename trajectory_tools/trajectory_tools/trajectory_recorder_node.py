import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from trajectory_tools_interfaces.srv import StopRecording 
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped

class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajecotry_recorder')

        self.get_logger().info("Node started")
        self.start_client = self.create_client(Trigger, 'start_recording')
        self.stop_client = self.create_client(StopRecording,'stop_recording')

        self.current_goal_id = None
        self.start_record = False
        self.start_after = False

        self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.nav_status_callback,
            10
        )

        self.activation_service = self.create_service(
            Trigger,
            'activate_trajectory_recorder',
            self.activate_callback
        )

        self.pause_recording = self.create_service(
            Trigger,
            '/pause_recording',
            self.pause_recording_callback
        ) 

        self.play_recording = self.create_service(
            Trigger,
            '/play_recording',
            self.play_recording_callback
        ) 
    def play_recording_callback(self, request, response):
        self.start_record = True
        if self.current_goal_id is not None:
            self.get_logger().info('End of the pause - Ready to record again')
            self.start_record_callback()
        response.success = True
        response.message = 'End of the pause - Ready to record again'

        return response



    def pause_recording_callback(self, request, response):
        self.start_record = False

        if self.current_goal_id is not None:
            self.get_logger().info('Recording session on pause')
            self.stop_record_callback()
            
        response.success = True
        return response

    def activate_callback(self, request, response):
        self.start_record = True
        self.get_logger().info('Trajectory recorder activated!')
        response.success = True
        return response

    def start_record_callback(self):
        future = self.start_client.call_async(Trigger.Request())
        self.get_logger().info("Recording request sent")
        self.start_record = True

        future.add_done_callback(self.on_start_response)


    def on_start_response(self, future):
        response = future.result()
        if response.success:
            self.get_logger().info("Record started !")
        else:
            self.get_logger().info("Record failed !")

        

    def stop_record_callback(self, start_after=False):
        future = self.stop_client.call_async(StopRecording.Request())
        future.add_done_callback(lambda f: self.on_stop_response(f, start_after))

    def on_stop_response(self, future, start_after):
        response = future.result()
        if response is not None and response.success:
            if start_after:
                self.start_record_callback()



    
    def nav_status_callback(self,msg):
        if len(msg.status_list)==0 or not self.start_record:
            return
        
        latest = msg.status_list[-1]
        status = latest.status
        goal_id = bytes(latest.goal_info.goal_id.uuid)

        if status == 2:
            if self.current_goal_id == goal_id:
                return


            if self.current_goal_id is not None:
                self.get_logger().info("Goal aborded")
                self.stop_record_callback(start_after=True)

            self.get_logger().info("Navigation start")
            self.current_goal_id = goal_id 
            self.start_record_callback()
 
        elif status == 4 and self.current_goal_id == goal_id:
            self.get_logger().info("Goal reached")
            self.get_logger().info("Recording stopped")
            self.stop_record_callback()
        
        elif status == 6:

            self.stop_record_callback()


            


    def destroy_node(self):

        self.get_logger().info("Shutting down node...")
        super().destroy_node()

       
def main(args=None):

    rclpy.init(args=args)
    node = TrajectoryRecorder()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received")

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

