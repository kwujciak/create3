import rclpy
from rclpy.node import Node
from irobot_create_msgs.srv import RobotPower

class RobotPowerService(Node):
    def __init__(self):
        super().__init__('robot_power_service')
        self.service = self.create_service(RobotPower, '/robot_power', self.turn_off_robot)
        self.future = None

    def turn_off_robot(self, request, response):
        print('here')
        self.get_logger().info('Turning off the robot')
        
        response.success = True
        response.message = 'Robot turned off successfully'

        self.future.set_result(response)
        print(2)

def main(args=None):
    rclpy.init(args=args)
    robot_power_service = RobotPowerService()
    print(1)

    robot_power_service.future = robot_power_service.create_service_future(RobotPower.Response)
    rclpy.spin_until_future_complete(robot_power_service, robot_power_service.future)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
