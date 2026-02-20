#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gpiozero import CPUTemperature
from example_interfaces.msg import Float64

class TemperatureSensorNode(Node):
    """
    A ROS2 node that reads the CPU temperature using the gpiozero library 
    and publishes it to a topic named "rpi_temperature".
    The node initializes a publisher for the temperature data 
    and sets up a timer to periodically read the CPU temperature and publish it as a Float64 message.
    """
    def __init__(self):
        """
        Initializes the TemperatureSensorNode, sets up the publisher for temperature data,
        and initializes a timer to periodically read and publish the CPU temperature.
        """
        super().__init__("temperature_sensor")
        self.temperature_pub_ = self.create_publisher(Float64, "rpi_temperature", 10)
        self.temperature_timer = self.create_timer(1.0, self.publish_temperature)
        self.get_logger().info("Temperature sensor has been started")

    def publish_temperature(self):
        """
        A callback function that is called periodically (every 1 second) to read the CPU temperature
        and publish it.
        """
        msg = Float64()
        msg.data = CPUTemperature().temperature
        self.temperature_pub_.publish(msg)

def main(args=None):

    rclpy.init(args=args)
    node = TemperatureSensorNode()
    rclpy.spin(node)
    rclpy.shutdown()