import rclpy

from geometry_msgs.msg import Twist

HALF_DISTANCE_BETWEEN_WHEELS = 0.047
WHEEL_RADIUS = 0.0299

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

    def __cmd_vel_callback(self, msg):
        # extract from Twist speeds for both motors
        left_speed = msg.linear.x - msg.angular.z * HALF_DISTANCE_BETWEEN_WHEELS / WHEEL_RADIUS
        right_speed = msg.linear.x + msg.angular.z * HALF_DISTANCE_BETWEEN_WHEELS / WHEEL_RADIUS

        # set motor speed
        self.__left_motor.setVelocity(left_speed / WHEEL_RADIUS)
        self.__right_motor.setVelocity(right_speed / WHEEL_RADIUS)

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
