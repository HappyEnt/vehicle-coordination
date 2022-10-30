import rclpy

class AnchorDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        time_step = int(self.__robot.getBasicTimeStep())

        self.__gps = self.__robot.getDevice("gps")
        self.__gps.enable(time_step)

        self.__emitter = self.__robot.getDevice("emitter")
        self.__emitter.setChannel(1)

        self.__pastTime = self.__robot.getTime()
        self.__initialTimeout = 3

        rclpy.init(args=None)
        self.__node = rclpy.create_node('anchor_node')

        self.__node.get_logger().info('Anchor driver initialized')

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        dt = self.__robot.getTime() - self.__pastTime
        if (dt > 0.025 and self.__robot.getTime() > self.__initialTimeout):
            x_global = self.__gps.getValues()[0]
            y_global = self.__gps.getValues()[1]
            self.__pastTime = self.__robot.getTime()
