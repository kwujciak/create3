
'''
These statements allow the Node class to be used and actions to be performed. 
'''
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import time

'''
These statements import messages and actions.
'''
from irobot_create_msgs.action import DriveDistance
from irobot_create_msgs.action import RotateAngle

'''
Input your namespace here as a global variable. 
'''
namespace = 'JonSnow'
        

class RotateActionClient(Node):
    '''
    This is an action client. Action clients send goal requests to action servers,
    which sends goal feedback and results back to action clients. This action client
    tells the robot to turn 90 degrees at a speed of 0.15. Subclass of Node.
    '''

    def __init__(self):
        '''
        We initialize the class by calling the Node constructor then
        naming our node 'rotate_action_client'
        '''
        print('Initializing a new action server in order to rotate.')
        super().__init__('rotate_action_client')
        '''
        Here we initiate a new action server. We include where to add the action client
        (self), the type of action (RotateAngle), and the action name ('rotate_angle').
        '''
                
        self._action_client = ActionClient(self, RotateAngle, namespace + '/rotate_angle')

    def send_goal(self, angle, max_rotation_speed):

        goal_msg = RotateAngle.Goal()
        goal_msg.angle = angle 
        goal_msg.max_rotation_speed = max_rotation_speed
        '''
        This method waits for the action server to be available.
        '''
        print('Waiting for action server to be available...')
        self._action_client.wait_for_server()
        '''
        Sends a goal to the server.
        '''   
        print('Action server available. Sending rotate goal to server.')
        
        # _action_client.send_goal_async(goal_msg) method returns a future to a goal handle
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        '''
        Returns a future to a goal handle. We need to register a callback 
        for when the future is complete.
        '''        
        # we register a callback for when the future is complete:
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        '''
        For those of you who are experienced coders, 
        you may be familiar with what a callback is. A callback
        is something we "call back" to from another function. It's
        kind of like a message handler.
        
        We run this everytime a new goal is achieved. It
        executes accepted goals.
        
        A callback that is executed when the future is complete.
        The future is completed when an action server accepts or rejects the goal request.
        Since there will be no result, we can check and determine if the goal was rejected
        and return early. 
        '''
        print('Checking if goal was accepted or rejected...')
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        '''
        We can request to see if the goal request was accepted or rejected.
        Future will complete when the result is ready.
        This step is registering a callback (similar to that of the goal response).
        
        Goal handle so that we cna request the result.
        '''
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        '''
        We know the goal was sent, but now we want to know when it is completed.
        Here, we are logging the result sequence.
        '''
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        '''
        This shuts down the node.
        '''        
        print('Shutting down rotate action client node.')
        rclpy.shutdown()
        

def main(args=None): 
    angle = 6.28
    speed = 0.5
    '''
    Initializes ROS2 and creates an instance of 
    'RotateActionClient'
    '''
    rclpy.init(args=args)
    action_client = RotateActionClient()
    '''
    Sends a goal and waits until goal is done.
    '''

    action_client.send_goal(angle, speed)
    rclpy.spin(action_client)
    time.sleep(0.5)  


if __name__ == '__main__':
    main()
