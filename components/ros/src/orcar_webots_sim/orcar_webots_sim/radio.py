from rosgraph_msgs.msg import Clock
from orcar_interfaces.msg import TaggedRadioPacket, RadioPacket
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from std_msgs.msg import Header

from struct import pack, unpack
from random import randint

from numpy import sqrt

import rclpy


class RangingRadio:
    # The `init` method is called only once the driver is initialized.
    # You will always get two arguments in the `init` method.
    # - The `webots_node` argument contains a reference on a Supervisor instance.
    # - The `properties` argument is a dictionary created from the XML tags.
    def init(self, webots_node, properties):
        # Unfortunately, we cannot get an instance of the parent ROS node.
        # However, we can create a new one.
        # rclpy.init(args=None)
        self.__node = rclpy.create_node('ranging_radio')

        # generate random mac
        self.__mac = randint(0, 2**32-1)

        # This will print the parameter from the URDF file.
        #
        #     `{ 'parameterExample': 'someValue' }`
        #
        self.__node.get_logger().info('  - properties: ' + str(properties))

        # The robot property allows you to access the standard Webots API.
        # See: https://cyberbotics.com/doc/reference/robot
        self.__robot = webots_node.robot
        self.__node.get_logger().info('  - robot name: ' + str(self.__robot.getName()))
        self.__node.get_logger().info('  - basic timestep: ' + str(int(self.__robot.getBasicTimeStep())))

        self.__robot_name = self.__robot.getName()

        # for our ranging radio we will need both a webots emitter and receiver device
        self.__emitter  = self.__robot.getDevice('emitter')
        self.__receiver = self.__robot.getDevice('receiver')

        # set emitter & receiver channel to channel 0
        self.__emitter.setChannel(0)

        if self.__receiver != None:
            self.__receiver.enable(int(self.__robot.getBasicTimeStep()))
            self.__receiver.setChannel(0)

        # register topic 'range_measurements' for publishing data
        self.__range_measurements_publisher = self.__node.create_publisher(
            TaggedRadioPacket, '/{}/ranging_radio/receive_queue'.format(self.__robot.getName()), 10
        )

        # create topic where other nodes can push data into
        self.__range_measurements_subscriber = self.__node.create_subscription(
            RadioPacket, '/{}/ranging_radio/transmit_queue'.format(self.__robot.getName()), self.__transmit_queue_callback, 10
        )

    def __transmit_queue_callback(self, msg):
        # create data for sending over simulated radio
        data = pack("!I%dc" % len(msg.payload), self.__mac, *msg.payload)
        self.__emitter.send(data)

    # The `step` method is called at every step.
    def step(self):
        # The self.__node has to be spinned once in order to execute callback functions.
        rclpy.spin_once(self.__node, timeout_sec=0)

        while self.__receiver != None and self.__receiver.getQueueLength() > 0:
            data = self.__receiver.getData()
            length = self.__receiver.getDataSize()

            mac, payload = unpack("!I%ds" % (length-4), data)

            # create header with reception time step
            header = Header()
            header.stamp = self.__node.get_clock().now().to_msg() # TODO use simulation clock here?

            # recover RadioMessage from received bytes
            packet = RadioPacket()
            packet.payload = [item.to_bytes(1, byteorder='big') for item in list(payload)]

            # calculate range from radio rssi
            rssi = self.__receiver.getSignalStrength()
            distance = sqrt(1.0 / rssi)

            # create TaggedRadioPacket
            tagged_packet = TaggedRadioPacket()
            tagged_packet.sender_mac = mac
            tagged_packet.header = header
            tagged_packet.range = distance
            tagged_packet.packet = packet

            self.__range_measurements_publisher.publish(tagged_packet)

            if self.__receiver.getQueueLength() > 0:
                self.__receiver.nextPacket()
