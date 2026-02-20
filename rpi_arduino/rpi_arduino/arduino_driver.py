#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import serial
from example_interfaces.msg import Int64


class ArduinoDriverNode(Node):
    """
    A ROS2 node that interfaces with an Arduino over a serial connection.
    The node periodically sends a command to toggle an LED on the Arduino
    and publishes any incoming data from the Arduino to a topic named "arduino_data".
    The node initializes the serial connection and sets up two timers: 
    one for sending commands to the Arduino and another for reading incoming data from the Arduino.
    """
    def __init__(self, ser):
        """
        Initializes the ArduinoDriverNode, sets up the publisher for incoming data from the Arduino,
        and initializes timers for sending commands and reading data from the Arduino.
        """
        super().__init__("arduino_driver")
        self.ser = ser
        self.led_on_ = False
        self.serial_pub_ = self.create_publisher(Int64, "arduino_data", 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.serial_timer = self.create_timer(0.01, self.serial_callback)
        
    
    def serial_callback(self):
        """
        A callback function that checks for incoming data from the Arduino over the serial connection.
        If data is available, it reads the data, decodes it, 
        and publishes it to the "arduino_data" topic as an Int64 message.
        """
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').rstrip()
            msg = Int64()
            msg.data = int(line)
            self.serial_pub_.publish(msg)


    def timer_callback(self):
        """
        A callback function that is called periodically (every 1 second) to send a command to the Arduino.
        The command toggles the state of an LED on the Arduino.
        """
        cmd = "led:" + str(int(self.led_on_)) + "\n"
        self.ser.write(cmd.encode('utf-8'))
        self.led_on_ = not self.led_on_


def main(args=None):
    #Init serial
    while True:
        try:
            ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1.0)
            print("Successfully connected to serial port")
            time.sleep(1)
            ser.reset_input_buffer()
            break
        except serial.SerialException:
            print("Serial port not found. Retrying in 1 seconds...")
            time.sleep(1)


    rclpy.init(args=args)
    node = ArduinoDriverNode(ser) 
    rclpy.spin(node)
    ser.close()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
