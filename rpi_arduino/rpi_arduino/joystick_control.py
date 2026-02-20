#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import serial
from example_interfaces.msg import Int64
import pygame



class JoystickControlNode(Node):
    """
    A ROS2 node that reads input from a joystick using the pygame library 
    and publishes the angle of the joystick to a topic named "joy".
    The node initializes the joystick and continuously checks for joystick events.
    """
    def __init__(self):
        """
        Initializes the JoystickControlNode, sets up the publisher for joystick data,
        and initializes the pygame library to read joystick input.
        """
        super().__init__("joystick_control")
        self.serial_pub_ = self.create_publisher(Int64, "joy", 10)

        self.time_last_cmd_found = time.time() # Timer function control variable

        # Initialize pygame
        pygame.init()
        # Get count of joysticks
        joystick_count = pygame.joystick.get_count()

        if joystick_count == 0:
            print("No joysticks found.")
        else:
            # Initialize the first joystick
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            print(f"Initialized Joystick: {joystick.get_name()}")

            try:
                # Main loop to read input
                while True:
                    for event in pygame.event.get():
                        # Check for value of left/right movement of the left joystick (XBOX)
                        # Values typically range from -1.0 to 1.0
                        if event.type == pygame.JOYAXISMOTION and (event.axis == 0):                      
                            # Convert to 0-180 degrees
                            # Fully Left: 0 -> Fully Right: 180
                            angle = (event.value + 1) / 2 * 180 

                            # Only publish if a command hasn't been found in the last
                            # 0.1 seconds to prevent spamming the topic
                            time_now = time.time()
                            if time_now - self.time_last_cmd_found > 0.1:
                                self.serial_pub_.publish(Int64(data=int(angle)))
                                self.time_last_cmd_found = time_now

            except KeyboardInterrupt:
                print("Exiting")
                pygame.quit()


def main(args=None):
    rclpy.init(args=args)
    node = JoystickControlNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
