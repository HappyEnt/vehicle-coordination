import rclpy
from orcar_interfaces.msg import TaggedRadioPacket, RadioPacket
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from std_msgs.msg import Header
from struct import pack

class AnchorDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        time_step = int(self.__robot.getBasicTimeStep())

        self.__gps = self.__robot.getDevice("gps")
        self.__gps.enable(time_step)

        self.__emitter = self.__robot.getDevice("emitter")
        self.__emitter.setChannel(1)

        self.__pastTime = self.__robot.getTime()
        self.__initialTimeout = 1

        rclpy.init(args=None)
        self.__node = rclpy.create_node('anchor_node')

        self.__node.get_logger().info('Anchor driver initialized')

        self.__range_measurements_publisher = self.__node.create_publisher(
            RadioPacket, '/{}/ranging_radio/transmit_queue'.format(self.__robot.getName()), 10
        )

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        dt = self.__robot.getTime() - self.__pastTime
        if (dt > 0.025 and self.__robot.getTime() > self.__initialTimeout):
            x_global = self.__gps.getValues()[0]
            y_global = self.__gps.getValues()[1]
            self.__pastTime = self.__robot.getTime()

            # create header
            # header = Header()
            # header.stamp = self.__node.get_clock().now().to_msg() # TODO use simulation clock here?

            # # create particles
            # particles = PoseArray()
            # particles.header = header
            # particles.poses = [Pose(position=Point(x=x_global, y=y_global, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))]

            # # create final RadioMessage for transfer
            # radioMsg = RadioMessage()
            # radioMsg.node_id = self.__robot.getName()
            # radioMsg.particles = particles
            # radioMsg.header = header

            packet = RadioPacket()
            data = pack('hhl', 1, 2, 3) # will be replaced with protobuf
            # convert each item to bytes
            packet.payload = [item.to_bytes(1, byteorder='big') for item in list(data)]

            # push radioMsg into transmit_queue topic
            self.__range_measurements_publisher.publish(packet)
