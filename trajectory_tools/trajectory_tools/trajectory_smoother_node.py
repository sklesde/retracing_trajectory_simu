import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from trajectory_tools_interfaces.srv import TrajectorySmoother
import numpy as np
from nav_msgs.msg import Path as Path_msgs
from geometry_msgs.msg import PoseStamped



class TrajectorySmootherNode(Node):
    def __init__(self):
        super().__init__('trajectory_smoother')
        self.get_logger().info('Node started')

        self.alpha = 0.5
        self.N = 150
        self.lambda_smoother = 0.60
    
        self.get_trajectory_smoothed = self.create_service(
                                TrajectorySmoother,
                                '/get_trajectory_smoothed',
                                self.get_trajectory_smoothed_callback
                                )
        

    

    def get_trajectory_smoothed_callback(self, request, response):
        self.get_logger().info('Smoothing path started')

        output_data = request.input_data 
        response.success = False

        try:
            np_input_data = np.array([[p.pose.position.x, p.pose.position.y] for p in request.input_data])
            ls_data = self.laplacian_smoothing(np_input_data, self.N, self.lambda_smoother)
            output_data = self.convert_np_to_list(ls_data, request.input_data)
            
            response.output_data = output_data
            response.success = True
            self.get_logger().info('Smoothing completed')

        except Exception as e:
            response.success = False
            self.get_logger().warn(f'Error while smoothing: {e}')

        path_msg = Path_msgs()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.poses = output_data
        self.last_path = path_msg
        # self.get_logger().info('Affichage du chemin')
        
        return response

    def laplacian_smoothing(self, np_input_data, N, lambda_smoother):
        ls_data = np.copy(np_input_data)

        for i in range(N):
        
            for pose in range(1,len(ls_data)-1):
                ls_data[pose] = ls_data[pose] + lambda_smoother*((ls_data[pose-1]+ls_data[pose+1])/2-ls_data[pose])

        return ls_data

    def convert_np_to_list(self, smoothed_path, input_data):

        output_data = []
        
        for idx, p in enumerate(input_data):
            
            x_smoothed = float(smoothed_path[idx][0])
            y_smoothed = float(smoothed_path[idx][1])
                                
            smoothed_pose = PoseStamped()

            smoothed_pose.header.frame_id = "map"

            smoothed_pose.header.stamp = self.get_clock().now().to_msg()
        
            smoothed_pose.pose.position.x = x_smoothed
            smoothed_pose.pose.position.y = y_smoothed
            smoothed_pose.pose.position.z = p.pose.position.z  

            smoothed_pose.pose.orientation = p.pose.orientation

            output_data.append(smoothed_pose)
        

        return output_data
    


    def destroy_node(self):

        self.get_logger().info("Shutting down node...")
        super().destroy_node()

       
def main(args=None):

    rclpy.init(args=args)
    node = TrajectorySmootherNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received")

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

