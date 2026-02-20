#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import serial
from example_interfaces.msg import Int64, Float64
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
import enum

class ControlMethod(enum.Enum):
    """
    An enumeration class that defines the different control methods for the servo motor.
    The control methods include:
    - AUTOMATIC_SWEEP: The servo motor automatically sweeps between 0 and 180 degrees.
    - JOYSTICK_CONTROL: The servo motor is controlled by a joystick input, 
                        where the angle is determined by the position of the joystick.
    - TEMPERATURE_CONTROL: The servo motor is controlled by temperature readings, 
                           where the angle is determined by the temperature value received from a sensor.
    """
    AUTOMATIC_SWEEP = "automatic_sweep"
    JOYSTICK_CONTROL = "joystick_control"
    TEMPERATURE_CONTROL = "temperature_control"

class ServoControlNode(Node):
    def __init__(self, ser):
        """
        A ROS2 node that controls a servo motor connected to an Arduino via serial communication. 
        The node can operate in three different control methods: automatic sweep, joystick control, and temperature control. 
        The control method can be changed dynamically by updating the "control_method" parameter, 
        and the node will handle the necessary subscriptions and timers accordingly.
        """
        super().__init__("servo_control")
        self.ser = ser
        self.angle = 0
        self.going_up = True
        self.declare_parameter("control_method", "automatic_sweep")
        self.create_sweep_timer()
        self.add_on_set_parameters_callback(self.paramChangeCallback)
    
    def paramChangeCallback(self, params):
        """
        A callback function that is called whenever a parameter is changed. 
        It checks if the control_method parameter has been changed and 
        updates the control method accordingly. 
        It also handles the creation and destruction of timers and subscriptions 
        based on the selected control method.

            Args:
                params (list): A list of parameters that have been changed.
            Returns:
                SetParametersResult: A result object indicating whether the parameter change was successful.
        """
        # Output the value of the changed parameter to the console"
        result = SetParametersResult()
        self.get_logger().info("Param control_method changed! New value is %s" % params[0].value)

        # Check if the new value is a valid control method
        if params[0].value == ControlMethod.AUTOMATIC_SWEEP.value:
            print("Starting automatic servo sweep...")
            self.create_sweep_timer()
            result.successful = True
        elif params[0].value == ControlMethod.JOYSTICK_CONTROL.value:
            print("Starting joystick control...")
            self.createJoystickSubscription()
            result.successful = True
        elif params[0].value == ControlMethod.TEMPERATURE_CONTROL.value:
            print("Starting temperature control...")
            self.create_temp_subscription()
            result.successful = True

        return result
    

    def joystickCallback(self, msg):
        """
        A callback function that is called whenever a new joystick angle message is received.
        It checks if the received angle is valid (between 0 and 180 degrees) and 
        sends the corresponding command to the Arduino via the serial port.

            Args:
                msg (Int64): The message containing the joystick angle.
        """
        print("Received joystick angle: " + str(msg.data))
        # Check if the received angle is valid (0-180 degrees)
        if msg.data < 0 or msg.data > 180:
            print("Invalid angle received from joystick: " + str(msg.data))
            msg.data = max(0, min(180, msg.data))

        # Send the command to the Arduino via serial
        cmd = str(msg.data).encode('utf-8')
        self.ser.write(cmd)
        print(cmd)

    def create_sweep_timer(self):
        """
        A function that creates a timer for the automatic servo sweep control method.
        It first checks if there are any existing subscriptions or timers and destroys them before creating the new timer for the servo sweep. 
        This ensures that only one control method is active at a time.
        """
        # Checks for existing subscriptions and timers and 
        # destroys them before creating the new timer for the servo sweep
        if (hasattr(self, 'temp_sub_') and self.temp_sub_ is not None):
            print("Destroying temperature subscription...")
            self.destroy_subscription(self.temp_sub_)
            print("Destroyed temperature subscription.")
            self.temp_sub_ = None
        if (hasattr(self, 'joystick_sub_') and self.joystick_sub_ is not None):
            print("Destroying joystick subscription...")
            self.destroy_subscription(self.joystick_sub_)
            print("Destroyed joystick subscription.")
            self.joystick_sub_ = None

        # Create the timer for the servo sweep
        print("Creating servo sweep timer...")
        self.timer = self.create_timer(0.01, self.servo_sweep)
        print("Servo sweep timer created.")

    def create_temp_subscription(self):
        """
        A function that creates a subscription for the temperature control method.
        It first checks if there are any existing subscriptions or timers and destroys them before creating the new subscription for the temperature control. 
        This ensures that only one control method is active at a time
        """
        # Checks for existing subscriptions and timers and 
        # destroys them before creating the new subscription for the temperature control
        if hasattr(self, 'timer'):
            print("Destroying servo sweep timer...")
            self.timer.cancel()
            print("Destroyed servo sweep timer.")
        if (hasattr(self, 'joystick_sub_') and self.joystick_sub_ is not None):
            print("Destroying joystick subscription...")
            self.destroy_subscription(self.joystick_sub_)
            print("Destroyed joystick subscription.")
            self.joystick_sub_ = None
        
        # Create the subscription for the temperature control
        print("Creating temperature subscription...")
        self.temp_sub_ = self.create_subscription(Float64, "rpi_temperature", self.temp_callback, 10)
        print("Temperature subscription created.")

    def createJoystickSubscription(self):
        """
        A function that creates a subscription for the joystick control method.
        It first checks if there are any existing subscriptions or timers and destroys them before creating the new subscription for the joystick control. 
        This ensures that only one control method is active at a time.
        """
        # Checks for existing subscriptions and timers and 
        # destroys them before creating the new subscription for the joystick control
        if hasattr(self, 'timer'):
            print("Destroying servo sweep timer...")
            self.timer.cancel()
            print("Destroyed servo sweep timer.")
        if (hasattr(self, 'temp_sub_') and self.temp_sub_ is not None):
            print("Destroying temperature subscription...")
            self.destroy_subscription(self.temp_sub_)
            print("Destroyed temperature subscription.")
            self.temp_sub_ = None
        
        # Create the subscription for the joystick control
        print("Creating joystick subscription...")
        self.joystick_sub_ = self.create_subscription(Int64, "joy", self.joystickCallback, 10)
        print("Joystick subscription created.")

    def servo_sweep(self):
        """
        A function that implements the automatic servo sweep control method.
        It continuously updates the servo angle between 0 and 180 degrees, 
        sending the corresponding command to the Arduino via the serial port. 
        The angle is incremented or decremented based on the current direction of movement (going_up)
        and reverses direction when the angle reaches the limits (0 or 180 degrees).
        """
        # Automatically sweeps the servo angle between 0 and 180 degrees, sending the corresponding command to the Arduino via serial
        try:
            # Control the direction of the sweep and reverse when limits are reached
            if self.angle >= 180:
                self.going_up = False
                # self.angle = 0
            elif self.angle == 0:
                self.going_up = True
            if self.going_up:
                self.angle += 1
            else:
                self.angle -= 1

            # Send the command to the Arduino via serial
            print("Setting servo angle to: " + str(self.angle))
            cmd = str(self.angle).encode('utf-8')
            self.ser.write(cmd)
            print(cmd)
        except serial.SerialException as e:
            print("Error writing to serial port: " + str(e))
            self.ser.close()
    
    def temp_callback(self, msg):
        """
        A callback function that is called whenever a new temperature message is received
        and sends the corresponding command to the Arduino via the serial port. 
            Args:
                msg (Float64): The message containing the temperature reading.
        """
        # Sends the corresponding command to the Arduino via serial 
        # whenever a new temperature message is received
        print("Received temperature: " + str(msg.data))
        cmd = str(msg.data).encode('utf-8')
        self.ser.write(cmd)
        print(cmd)


def main(args=None):
    #Init serial for Arduino communication
    while True:
        try:
            ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1.0)
            print("Successfully connected to Arduino serial port")
            time.sleep(1)
            ser.reset_input_buffer()
            break
        except serial.SerialException:
            print("Serial port not found. Retrying in 1 seconds...")
            time.sleep(1)

    #Spin the ROS node and destroy the serial connection on shutdown
    rclpy.init(args=args)
    node = ServoControlNode(ser)
    rclpy.spin(node)
    ser.close()
    print("Exiting...")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
