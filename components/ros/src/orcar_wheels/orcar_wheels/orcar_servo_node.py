import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from rpi_hardware_pwm import HardwarePWM

from math import pi

HALF_DISTANCE_BETWEEN_WHEELS = 0.047
WHEEL_RADIUS = 0.0299
WHEEL_DIAMETER = 2 * pi * WHEEL_RADIUS

BASE_FREQUENCY = 50.0
BASE_PERIOD = 1000.0 / BASE_FREQUENCY

MAX_RPM = 110.0 # from datasheet, could deviate in practice depending on the load and supply voltage
MAX_VELOCITY = WHEEL_DIAMETER * MAX_RPM / 60.0

FULL_BACKWARD_DUTY_CYCLE   = 1.0 / BASE_PERIOD * 100
STOP_DUTY_CYCLE            = 1.5 / BASE_PERIOD * 100
FULL_FORWARD_DUTY_CYCLE = 2.0 / BASE_PERIOD * 100

class OrcarServoNode(Node):
    def __init__(self):
        super().__init__("orcar_servo_driver")
        self.pwm_left = HardwarePWM(pwm_channel=0, hz=50)
        self.pwm_right = HardwarePWM(pwm_channel=1, hz=50)
        self.pwm_left.start(STOP_DUTY_CYCLE) # full duty cycle
        self.pwm_right.start(STOP_DUTY_CYCLE) # full duty cycle

        # create cmd_vel topic for receiving Twist messages
        cmd_vel_subscription = self.create_subscription(Twist, '~/cmd_vel', self.__cmd_vel_callback, 1)

    def setVelocity(self, left_velocity, right_velocity):
        # Define maximum velocity
        fraction_left = left_velocity / MAX_VELOCITY
        fraction_right = right_velocity / MAX_VELOCITY

        # clamp values
        fraction_left = max(min(fraction_left, 1.0), -1.0)
        fraction_right = max(min(fraction_right, 1.0), -1.0)

        # set motor speed
        self.pwm_left.change_duty_cycle(STOP_DUTY_CYCLE  + (FULL_FORWARD_DUTY_CYCLE - STOP_DUTY_CYCLE) * fraction_left)
        self.pwm_right.change_duty_cycle(STOP_DUTY_CYCLE  + (FULL_FORWARD_DUTY_CYCLE - STOP_DUTY_CYCLE) * fraction_right)

    def __cmd_vel_callback(self, msg):
        # extract from Twist speeds for both motors
        left_vel  = (msg.linear.x - msg.angular.z * HALF_DISTANCE_BETWEEN_WHEELS)
        right_vel = (msg.linear.x + msg.angular.z * HALF_DISTANCE_BETWEEN_WHEELS)

        self.setVelocity(left_vel, right_vel)



def main(args=None):
    rclpy.init(args=None)

    orcar_servo_node = OrcarServoNode()
    rclpy.spin(orcar_servo_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    orcar_servo_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
