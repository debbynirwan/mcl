import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


import pick
import struct
import serial.tools.list_ports
import itertools
import serial

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
        self.create_subscription(Odometry, '/odom', self.callback_odom, 1)
        self.create_subscription(LaserScan, '/scan', self.callback_laser, 1)
        # Initialize serial port
        self._serial_port: serial.Serial = serial.Serial(port, baud)
        self._odom_reading = Odometry()
        self._prev_odom_reading = self._odom_reading
        self._laser_reading = LaserScan()
        self._prev_laser_reading = self._laser_reading

    def callback_odom(self, msg: Odometry):
        """ Callback method for ROS messages. """
        self.get_logger().info(f"I heard: {msg.pose}")
        self._odom_reading = msg
        self.send_serial()


    def callback_laser(self, msg: LaserScan):
        """ Callback method for ROS messages. """
        self.get_logger().info(f"I heard: {msg.ranges}")
        self._odom_reading = msg
        self.send_serial()

    def send_serial(self):
        """ Send data to serial port if both odom and laser have changed. """
        # Guard clause to check that both readings have changed
        if self._prev_laser_reading == self._laser_reading or self._prev_odom_reading == self._odom_reading:
            # self.get_logger().info("ONE OF THE READINGS HAS NOT CHANGED YET")
            return
        # if self._prev_odom_reading is self._odom_reading:
        #     self.get_logger().info("UHM WHUT!")
        #     return
        # Update the old readings
        self._prev_laser_reading = self._laser_reading
        self._prev_odom_reading = self._odom_reading

        # Preparing the data to be sent out
        buff = prepare_data(self._odom_reading, self._laser_reading)
        self.get_logger().info(f"Sending {len(buff)} bytes: {buff}")
        # Send the data
        self._serial_port.write(buff)

def prepare_data(odom_reading: Odometry, laser_reading: LaserScan):
    """ Prepare data for sending to serial port. """
    # -------------------------- START AND END --------------------------
    # Start of message sequence
    # 8 bytes
    start = "ABCDEFGH"
    # End of message sequence
    # 8 bytes
    end = "HGFEDCBA"
    # Total: 16 bytes
    # -------------------------- Odometry data --------------------------
    # 3*8 = 24 bytes
    pose_position = [odom_reading.pose.pose.position.x, odom_reading.pose.pose.position.y, odom_reading.pose.pose.position.z]
    # 4*8 = 32 bytes
    pose_orientation = [odom_reading.pose.pose.orientation.x, odom_reading.pose.pose.orientation.y, odom_reading.pose.pose.orientation.z, odom_reading.pose.pose.orientation.w]
    # 36*8 = 288 bytes
    pose_covariance = list(odom_reading.pose.covariance)
    # 3*8 = 24 bytes
    twist_linear = [odom_reading.twist.twist.linear.x, odom_reading.twist.twist.linear.y, odom_reading.twist.twist.linear.z]
    # 3*8 = 24 bytes
    twist_angular = [odom_reading.twist.twist.angular.x, odom_reading.twist.twist.angular.y, odom_reading.twist.twist.angular.z]
    # 36*8 = 288 bytes
    twist_covariance = list(odom_reading.twist.covariance)
    # Total: 680 bytes
    odom_data = [pose_position, pose_orientation, pose_covariance, twist_linear, twist_angular, twist_covariance]
    # -------------------------- Laser data -----------------------------
    # 1*8 = 8 bytes
    laser_angle_min = [laser_reading.angle_min]
    # 1*8 = 8 bytes
    laser_angle_max = [laser_reading.angle_max]
    # 1*8 = 8 bytes
    laser_angle_increment = [laser_reading.angle_increment]
    # 1*8 = 8 bytes
    laser_time_increment = [laser_reading.time_increment]
    # 1*8 = 8 bytes
    # laser_scan_time = [laser_reading.scan_time]
    # 1*8 = 8 bytes
    laser_range_min = [laser_reading.range_min]
    # 1*8 = 8 bytes
    laser_range_max = [laser_reading.range_max]
    # FOR TESTING IM CREATING MY OWN FIXED LENGTH LIST OF RANGES AND INTENSITIES SINCE LaserScan RETURNS AN EMPTY LIST
    # laser_ranges = list(laser_reading.ranges)
    # 8*8 = 64 bytes
    laser_ranges = [0.0 for i in range(8)]
    # laser_intensities = list(laser_reading.intensities)
    # 8*8 = 64 bytes
    laser_intensities = [0.0 for i in range(8)]
    # Total: 176 (or 184) bytes (depending on if scan_time is included)
    laser_data = [laser_angle_min, laser_angle_max, laser_angle_increment, laser_time_increment, laser_range_min, laser_range_max, laser_ranges, laser_intensities]
    # -------------------------- Serial data ----------------------------
    # Pack the data into a byte string
    buff = struct.pack(f"{len(start)}s", start.encode("ASCII"))
    buff += struct.pack(f"{sum([len(item) for item in odom_data])}d", *list(itertools.chain.from_iterable(odom_data)))
    buff += struct.pack(f"{sum([len(item) for item in laser_data])}d", *list(itertools.chain.from_iterable(laser_data)))
    buff += struct.pack(f"{len(end)}s", end.encode("ASCII"))
    return buff

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