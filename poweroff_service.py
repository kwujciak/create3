import rclpy
from rclpy.node import Node
from irobot_create_msgs.srv import RobotPower

namespace = '/Moondancer'

class RobotPowerService(Node):
    def __init__(self):
        super().__init__('robot_power_service')
        self.service = self.create_service(RobotPower, namespace + '/robot_power', self.turn_off_robot)
        self.future = None  # Initialize the future here

    def turn_off_robot(self, request, response):
        print('here')
        self.get_logger().info('Turning off the robot')
        
        # Populate the response (optional)
        response.success = True
        response.message = 'Robot turned off successfully'

        # Signal that the service has completed
        self.future.set_result(response)

def main(args=None):
    rclpy.init(args=args)
    robot_power_service = RobotPowerService()

    # Create a future and store it in the service
    robot_power_service.future = robot_power_service.create_service_future(RobotPower.Response)

    # Spin until the service is called
    rclpy.spin_until_future_complete(robot_power_service, robot_power_service.future)

    # Shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()
