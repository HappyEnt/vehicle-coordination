import rclpy
from orcar_interfaces.msg import TaggedRadioPacket, RadioPacket
from orcar_interfaces.Particles_pb2 import Particle, ParticleArray
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
        if (dt > 0.5 and self.__robot.getTime() > self.__initialTimeout):
            x_global = self.__gps.getValues()[0]
            y_global = self.__gps.getValues()[1]
            self.__pastTime = self.__robot.getTime()

            # first create single Particle from protobuf
            particle = Particle()
            particle.x = x_global
            particle.y = y_global
            particle.weight = 1.0

            # create protobuf ParticleArray with exactly one Particle
            particle_array = ParticleArray()
            particle_array.particles.extend([particle])

            # create packet for radio module
            packet = RadioPacket()

            # convert each item to bytes
            packet.payload = [item.to_bytes(1, byteorder='big') for item in list(particle_array.SerializeToString())]

            # push radioMsg into transmit_queue topic
            self.__range_measurements_publisher.publish(packet)
