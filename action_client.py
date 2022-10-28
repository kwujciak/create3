
'''
These statements allow the Node class to be used and actions to be performed. 
'''

#if we go to terminal we can first see a list of potential actions for the robot by doing ros2 action list -t
#this tells us all the possible actions and the action types
#lets start today like we always do, making a python file we'll name it actionclient_rotate.py 
#because this script will make the create3 rotate
#then we'll start commenting the title of the file so we remember what it is

#lets do our usual import statements of rclpy, rclpy.node and then today we also need to import
#the action client 

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

'''
Then we'll import our action message rotate angle, agian if we go to irobot create 3 messages github we can 
get information on the action types, their messages and what information they require
'''
from irobot_create_msgs.action import RotateAngle

'''
Input your namespace here as a global variable. 
'''
namespace = 'JonSnow'
        

# we'll start a new class for our action client. action clients work in conjunction with action servers. the action client will 
#send a goal request to the server, which will preform the action and then send back feedback and results to the client.
# this action client will tell the robot to turn at a given angle and speed. 

class RotateActionClient(Node):

    def __init__(self):
        '''
        We initialize the class by calling the Node constructor then
        naming our node 'rotate_action_client'
        '''
        super().__init__('rotate_action_client')
        '''
        Here we initiate a new action client. We include where to add the action client
        (self), the type of action (RotateAngle), and the action name ('rotate_angle') with our namespace.
        '''
                
        self._action_client = ActionClient(self, RotateAngle, namespace + '/rotate_angle')

        #then lets define a new function to send the goal to the action server
        #we'll make angle and max rotation speed arguments for the function
    def send_goal(self, angle, max_rotation_speed):

        #then lets define our goal message to be that of the of the rotate angle action type
        #this will allow us to use the dot operator on the goal message to define components of the message
        goal_msg = RotateAngle.Goal()
        
        #we'll use the dot operator to set the angle equal to the argument that comes into the function
        #and the same for the max rotation speed
        
        #we know that the goal message for the rotate action 
        goal_msg.angle = angle 
        goal_msg.max_rotation_speed = max_rotation_speed
        
        '''
        This method waits for the action server to be available. 
        '''
        print('Waiting for action server to be available...')
        self._action_client.wait_for_server()
        
        
        # we'll send the goal message asynchronously
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        
        
        # we register a callback for when the future is complete.
        # a future is complete when the action server accepts or rejects
        # goal request.
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        
    def goal_response_callback(self, future): 
       '''
       For those of you who are experienced coders, 
       you may be familiar with what a callback function is. A callback
       is something we intuitively "call back" to from another function. 
       It's kind of like a message handler.
       
       The point of a callback function is to execute accepted goals.
       
       If you look back to this line (79), we register this callback 
       for when the future is complete.
       Just for reference, the future is "complete" when the server 
       accepts or rejects the goal. The goal in our case is rotating
       the create 360 degrees. 
       Thus, this callback is executed when the future is complete.
       
       For our specific case, we want a callback because then we'll know 
       when the goal that was sent (as Maddie showed) is accepted.
       '''
      
       print('Checking if goal was accepted or rejected...')
        
       '''   
       This function runs everytime a new goal is achieved.
       It then executes accepted goals.  
       
       Now, we want a goal handle for the goal we sent. A goal handle
       is used to keep track of the status of the goal so we can get 
       the final result. We'll use this handle to see the result. 
       '''
       goal_handle = future.result()
       if not goal_handle.accepted:
           self.get_logger().info('Goal rejected :(')
           return     
       self.get_logger().info('Goal accepted :)')

        '''
        Now, we'll use the goal handle so that we can request the result.
        The result is either accepted or rejected. 
        '''
        self._get_result_future = goal_handle.get_result_async()
        
        '''
        When the result is ready, we'll get a future that will complete. 
        So, just like the goal response, we'll register another callback. 
        '''
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        '''
        We know the goal was sent, but now we want to know when it is completed.
        Here, we are logging the result sequence.
        '''
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        '''
        Now, we'll shut down the node to avoid any issues.
        '''
        print('Shutting down rotate action client node.')
        rclpy.shutdown()
        '''
        That concludes our use with callback functions, and
        we're almost done with our action client script.
        '''
        

def main(args=None): 
    '''
    We'll set our desired angle of rotation
    and our rotational speed, not to be confused
    with max angle and speed defined in the send_goal function. 
    '''
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

'''
We'll close this script how we always do,
by running this main function. 
'''
if __name__ == '__main__':
    main()
