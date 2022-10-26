''' 
action_drive_square_2.py
Tufts Create®3 Educational Robot Example
by Maddie Pero

This is an example of combining two action clients in one class. To learn how to write mulitple classes
in one script reference action_drive_square
'''

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

'''
import messages/actions
'''
from irobot_create_msgs.action import DriveDistance
from irobot_create_msgs.action import RotateAngle

'''
Input your namespace here as a global variable. 
'''
namespace = '[Namespace]'

class SquareActionClient(Node):
    ''' 
    This is an action client. It sends a goal to an action server which sends
    back feedback and the result of the goal. The goal of this action client is to
    drive and then turn the Create 3 to make a square. Here we are defining a class
    'SquareActionClient' which is a subclass of Node.
    '''
    
    def __init__(self):
        
        '''First we initialize the class by calling the node constructor & 
        naming our node 'sqaure_action_client'
        '''        
        super().__init__('square_action_client')
        
        '''
        Then we initilaize two action clients. One to drive the robot and one to turn it. 
        We include where to add the action client (self), the type of action, and the name
        of the action 
        '''
        print('Initializing a new action server in order to rotate.')
        self.turn = ActionClient(self, RotateAngle, namespace + '/rotate_angle')
  
        '''
        Below we initialize a counter that we will call later so that 
        the robot only drives one square.
        '''
        
    def send_turn(self):
        '''
        Here we repeat the same process but to turn the corner of the square.
        '''
        goal_msg_turn = RotateAngle.Goal()
        goal_msg_turn.angle = 1.57
        goal_msg_turn.max_rotation_speed = 1.0
        
        print('Waiting for action server to be available...')
        self.turn.wait_for_server()
        print('Action server available. Sending turn goal to server.')
        self._send_goal_future = self.turn.send_goal_async(goal_msg_turn)
        self._send_goal_future.add_done_callback(self.turn_response_callback)
        
    def turn_response_callback(self, future):
        ''' 
        Again we repeat the process but for the corner turns
        '''
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.turn_result_callback)
        
    def turn_result_callback(self, future):
        ''' 
        Again we log the result.
        '''
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        
def main(args=None):
    '''
    Initializes ROS2 and creates an instance of 
    'SquareActionClient'
    '''
    rclpy.init(args=args)
    square_action_client = SquareActionClient() 
    square_action_client.send_turn()
    '''
    Sends the first goal, waits until goal is done & shuts down the node if there is 
    an excception.
    '''
    try:
        rclpy.spin(square_action_client)
    except KeyboardInterrupt:
        print('\nCaught keyboard interrupt')
        rclpy.shutdown()
    finally:
        print("Done")


if __name__ == '__main__':
    main()
