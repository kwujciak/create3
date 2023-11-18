import rclpy
from rclpy.node import Node
from irobot_create_msgs.srv import RobotPower

namespace = '/Moondancer'

class RobotPowerService(Node):
    def __init__(self):
        super().__init__('robot_power_service')
        self.service = self.create_service(RobotPower, namespace + '/robot_power', self.turn_off_robot)

    def turn_off_robot(self, response):
        #def turn_off_robot(self, request, response):
        print('here')
        
        self.get_logger().info('Turning off the robot')
        
        # Populate the response (optional)
        response.success = True
        response.message = 'Robot turned off successfully'

        return response


def main(args=None):
    rclpy.init(args=args)
    robot_power_service = RobotPowerService()
    future = robot_power_service.turn_off_robot(response)
    print(future)
    rclpy.spin_until_future_complete(robot_power_service, future)
    #rclpy.shutdown()
    #rclpy.spin(robot_power_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
