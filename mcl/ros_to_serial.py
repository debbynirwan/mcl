import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


import pick
import serial.tools.list_ports

class ROSSerialInterface(Node):
    """
    ROS to Serial interface node.

    ...

    Attributes
    ----------
    port : serial.Serial
        COM/tty port on which to send data
    baud : int
        baud rate for serial communication
    """
    def __init__(self, port: serial.Serial, baud: int):
        super().__init__('ros_serial_interface')
        # Create subscriptions
        self.create_subscription(Odometry, '/odom', self.callback, 1)
        self.create_subscription(LaserScan, '/scan', self.callback, 1)
        # Initialize serial port
        self._serial_port = serial.Serial(port, baud)

    def callback(self, msg):
        """ Callback method for ROS messages. """
        self.get_logger().info(f"I heard: {msg.data}")
        self._serial_port(msg.data.encode('utf-8'))

def main(args=None):
    # Get list of available serial port
    ports = serial.tools.list_ports.comports()
    port_list = []
    for port, desc, _ in ports:
        port_list.append((port, desc))
    # Pick port from list
    port = pick.pick(port_list, 'Select serial port:')[0][0]
    # Pick a baudrate
    baud = pick.pick([9600, 19200, 38400, 57600, 115200], 'Select baudrate:')[0]
    # Create the ROS node
    rclpy.init(args=args)

    ros_serial_interface = ROSSerialInterface(port, baud)

    rclpy.spin(ros_serial_interface)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ros_serial_interface.destroy_node()
    rclpy.shutdown()