import rclpy, os, sys
from Subs.ROS2Lib import Drive, Audio
import time

class Create():
    def __init__ (self, namespace = ''):
        rclpy.init(args = None)
        self.namespace = namespace
        self.drive_client = Drive(namespace)
        self.audio_publisher = Audio(namespace)
        
        print('ros domain: ' + str(os.environ['ROS_DOMAIN_ID']))
        print('middleware: ' + str(os.environ['RMW_IMPLEMENTATION']))
        time.sleep(1)

    def beep(self, frequency = 440):
        '''
        Beeps
        '''
        print('publish beep ', end = '')
        self.audio_publisher.beep(frequency)
        time.sleep(1)
        print('done')
            
    def forward(self,dist = 0.5):
        '''
        Goes the distance and then stops the ROS2 connection
        '''
        speed = 0.25
        print('forward %0.2f: goal' % dist, end = '')
        self.drive_client.set_goal(float(dist),speed)
        print(' set ', end = '')
        self.wait(self.drive_client)
        print('done')

    def wait(self, client):
        rclpy.spin_once(client)
        while not client.done:
            #time.sleep(0.1)
            print('...', end = '')
            rclpy.spin_once(client)
            
    def close(self):
        print('closing ', end = '')
        self.drive_client.destroy_node()
        self.rotate_client.destroy_node()
        self.led_publisher.destroy_node()
        self.audio_publisher.destroy_node()
        rclpy.shutdown()
        print('done')
