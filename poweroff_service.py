import rclpy
from rclpy.node import Node
from irobot_create_msgs.srv import RobotPower

class RobotPowerService(Node):
    def __init__(self):
        super().__init__('robot_power_service')
        self.service = self.create_service(RobotPower, '/robot_power', self.turn_off_robot)

    def turn_off_robot(self, request, response):
        # Assuming that turning off the robot involves some specific actions
        # You can add your implementation here

        # For now, let's print a message
        self.get_logger().info('Turning off the robot')
        
        # Populate the response (optional)
        response.success = True
        response.message = 'Robot turned off successfully'

        return response

def main(args=None):
    rclpy.init(args=args)
    robot_power_service = RobotPowerService()
    rclpy.spin(robot_power_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
