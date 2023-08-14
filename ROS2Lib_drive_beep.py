
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import time

from irobot_create_msgs.action import DriveDistance
from irobot_create_msgs.msg import AudioNoteVector 
from irobot_create_msgs.msg import AudioNote 
from builtin_interfaces.msg import Duration  
import geometry_msgs.msg

class Audio(Node):
  
    def __init__(self, namespace = '/Happy'):   
        super().__init__('audio_publisher')
        self.audio_publisher = self.create_publisher(AudioNoteVector, namespace + '/cmd_audio', 10)
        self.audio = AudioNoteVector()
        self.append = False
        
    def beep(self, frequency = 440):
        self.audio.notes = [AudioNote(frequency = frequency, max_runtime = Duration(sec = 1, nanosec = 0))]
        self.audio_publisher.publish(self.audio)

class Drive(Node):

    def __init__(self, namespace = '/Happy'):
        self.done = False
        super().__init__('drive_distance_action_client')
        self._action = ActionClient(self, DriveDistance, namespace + '/drive_distance')
        
    def set_goal(self, distance = 0.5, speed = 0.15):
        self.done = False
        goal_msg = DriveDistance.Goal()
        goal_msg.distance = distance
        goal_msg.max_translation_speed = speed
        self._action.wait_for_server()  # Wait for the server to be available and then send the goal.
        self._send_goal_future = self._action.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_request_callback)

    def goal_request_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.done = True
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):    
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        self.done = True
