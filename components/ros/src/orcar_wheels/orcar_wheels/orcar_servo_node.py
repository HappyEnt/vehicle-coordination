import rclpy

from geometry_msgs.msg import Twist
from rpi_hardware_pwm import HardwarePWM

HALF_DISTANCE_BETWEEN_WHEELS = 0.047
WHEEL_RADIUS = 0.0299

class OrcarServoNode(Node):
    def __init__(self):
        pwm = HardwarePWM(pwm_channel=0, hz=60)

        rclpy.init(args=None)
        self.__node = rclpy.create_node('orcar_wheels_servo')

        # create cmd_vel topic for receiving Twist messages
        self.__node.create_subscription(Twist, '/{}/cmd_vel'.format(self.__robot.getName()), self.__cmd_vel_callback, 1)

    def __cmd_vel_callback(self, msg):
        # extract from Twist speeds for both motors
        left_speed = msg.linear.x - msg.angular.z * HALF_DISTANCE_BETWEEN_WHEELS / WHEEL_RADIUS
        right_speed = msg.linear.x + msg.angular.z * HALF_DISTANCE_BETWEEN_WHEELS / WHEEL_RADIUS

        # set motor speed

def main(args=None):
    rclpy.init(args=args)

    orcar_servo_node = OrcarServoNode()

    rclpy.spin(orcar_servo_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    orcar_servo_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
