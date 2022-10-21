import rclpy

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

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
        self.__node = rclpy.create_node('orcar_driver')

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
