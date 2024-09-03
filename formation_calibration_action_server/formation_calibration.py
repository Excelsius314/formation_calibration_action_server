
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import time

from epuck_driver_interfaces.action import SimpleMovement
from epuck_driver_interfaces.action import CalibrateFormation


from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import SetBool
from rclpy.action import ActionClient



class FormationCallibrator(Node):

    def __init__(self):
        super().__init__('sound_data_collection_action_server')
        self._action_servers = ActionServer(self, CalibrateFormation, 'formationCalibrator', self.execute_goal_callback)

        self.convergence_threshhold = self.declare_parameter('convergence_threshhold', value=0.05).get_parameter_value().double_value

        #self.data_cb_group = MutuallyExclusiveCallbackGroup()
        self.robot_names = self.declare_parameter('robot_names', []).get_parameter_value().string_array_value

        self.robot_centering_srv_clients = {robot: self.create_client(SetBool, robot + "/image_centering/activation") for robot in self.robot_names}

        for cli in self.robot_centering_srv_clients.values():
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Service of {} not available, waiting again...'.format(cli))

        self.srv_req = SetBool.Request()
        self.robot_distance_calibration_clients = {robot: ActionClient(self, SimpleMovement, "{}/movement_goal".format(robot)) for robot in self.robot_names}
    

        
    def execute_goal_callback(self, goal_handle):


        robot_distance = goal_handle.request.robot_distance
        
        max_correction = -1

        while max_correction >= self.convergence_threshhold:

            max_correction = -1
            for robot in self.robot_names:
                
                self.srv_req.data = True
                self.robot_centering_srv_clients[robot].send_goal(self.req)
                time.sleep(3)
                self.srv_req.data = False
                self.robot_centering_srv_clients[robot].send_goal(self.req)

                goal_msg = SimpleMovement.Goal()
                goal_msg.tof_approach = True
                goal_msg.distance = robot_distance

                client_goal_handle_f= self.robot_distance_calibration_clients[robot].send_goal(goal_msg)
                result_msg = client_goal_handle_f.get_result()

                max_correction = max(max_correction, result_msg.distance_driven)
        
        goal_handle.succeed()
        
        res_msg = CalibrateFormation.Result()
        res_msg.dummy = True
        return res_msg
        


    
    
def main(args=None):
    rclpy.init(args=args)

    formation_callibrator_action_server = FormationCallibrator()

    executor = MultiThreadedExecutor()
    executor.add_node(formation_callibrator_action_server)

    try:
        formation_callibrator_action_server.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        formation_callibrator_action_server.get_logger().info('Keyboard interrupt, shutting down.\n')
    
    formation_callibrator_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

