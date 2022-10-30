from rosgraph_msgs.msg import Clock
from orcar_interfaces.msg import RadioMessage
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from std_msgs.msg import Header
import rclpy


class RangingRadio:
    # The `init` method is called only once the driver is initialized.
    # You will always get two arguments in the `init` method.
    # - The `webots_node` argument contains a reference on a Supervisor instance.
    # - The `properties` argument is a dictionary created from the XML tags.
    def init(self, webots_node, properties):
        # Unfortunately, we cannot get an instance of the parent ROS node.
        # However, we can create a new one.
        rclpy.init(args=None)
        self.__node = rclpy.create_node('plugin_node_example')

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

        # for our ranging radio we will need both a webots emitter and receiver device
        self.__emitter  = self.__robot.getDevice('emitter')
        self.__receiver = self.__robot.getDevice('receiver')

        # register topic 'range_measurements' for publishing data
        self.__range_measurements_publisher = self.__node.create_publisher(
            RangeMeasurements, 'range_measurements', 10)

    # The `step` method is called at every step.
    def step(self):
        # The self.__node has to be spinned once in order to execute callback functions.
        rclpy.spin_once(self.__node, timeout_sec=0)

        # create header
        header = Header()
        header.stamp = self.__node.get_clock().now().to_msg() # TODO use simulation clock here?

        # create particles
        particles = PoseArray()
        particles.header = header
        particles.poses = [Pose(position=Point(x=0.0, y=0.0, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))]

        # create final RadioMessage for transfer
        radioMsg = RadioMessage()
        radioMsg.node_id = "test_id"
        radioMsg.particles = particles
        radioMsg.header = header

        self.__range_measurements_publisher.publish(radioMsg)
