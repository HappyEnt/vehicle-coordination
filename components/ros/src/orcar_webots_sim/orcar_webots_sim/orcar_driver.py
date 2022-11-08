import rclpy

from geometry_msgs.msg import Twist

from math import pi

HALF_DISTANCE_BETWEEN_WHEELS = 0.047
WHEEL_RADIUS = 0.03
WHEEL_DIAMETER = 2 * pi * WHEEL_RADIUS

MAX_VELOCITY = 25 # in radians per second, see webots Motor Documentation
MAX_VELOCITY_MS = MAX_VELOCITY / pi * WHEEL_DIAMETER

class OrcarDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__left_motor = self.__robot.getDevice('left_wheel_hinge')
        self.__right_motor = self.__robot.getDevice('right_wheel_hinge')

        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)

        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        rclpy.init(args=None)
        self.__node = rclpy.create_node('orcar')

        # create cmd_vel topic for receiving Twist messages
        self.__node.create_subscription(Twist, '/{}/cmd_vel'.format(self.__robot.getName()), self.__cmd_vel_callback, 1)

    def setVelocity(self, left_velocity, right_velocity):
        # Define maximum velocity
        fraction_left = left_velocity / MAX_VELOCITY_MS
        fraction_right = right_velocity / MAX_VELOCITY_MS

        # clamp values
        fraction_left = max(min(fraction_left, 1.0), -1.0)
        fraction_right = max(min(fraction_right, 1.0), -1.0)

        # set motor speed
        self.__left_motor.setVelocity(MAX_VELOCITY * fraction_left)
        self.__right_motor.setVelocity(MAX_VELOCITY * fraction_right)

    def __cmd_vel_callback(self, msg):
        # extract from Twist speeds for both motors
        left_vel = msg.linear.x - msg.angular.z * HALF_DISTANCE_BETWEEN_WHEELS
        right_vel = msg.linear.x + msg.angular.z * HALF_DISTANCE_BETWEEN_WHEELS

        # set motor speed
        self.setVelocity(left_vel, right_vel)

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
