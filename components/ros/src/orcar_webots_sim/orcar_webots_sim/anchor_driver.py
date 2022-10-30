import rclpy

class AnchorDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__gps = self.__robot.getDevice("gps")
        self.__gps.enable(TIME_STEP)

        self.__emitter = self.__robot.getDevice("emitter")
        self.__emitter.setChannel(1)

        self.__pastTime = robot.getTime()
        self.__initialTimeout = 3

        rclpy.init(args=None)
        self.__node = rclpy.create_node('anchor_node')

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        dt = self.__robot.getTime() - pastTime
        if (dt > 0.025 and self.__robot.getTime() > self.__initialTimeout):
            x_global = self.__gps.getValues()[0]
            y_global = self.__gps.getValues()[0]

            print("gps values: ", x_global, y_global)


