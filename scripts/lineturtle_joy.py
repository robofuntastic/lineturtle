#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8

class JoystickController(Node):
    def __init__(self):
        super().__init__('lineturtle_joy')
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        self.leds_publisher = self.create_publisher(Int8, '/LEDs', 10)
        self.servo_publisher = self.create_publisher(Int8, '/servo', 10)
        self.servo_position = 0

    def joy_callback(self, msg):
        # Check if the message has enough buttons
        if len(msg.buttons) >= 4:
            button_A = msg.buttons[0]
            button_B = msg.buttons[1]
            button_X = msg.buttons[2]
            button_Y = msg.buttons[3]

            # Handle button presses and publish corresponding values on /LEDs topic
            if button_A == 1:
                self.publish_led_value(0)
            elif button_B == 1:
                self.publish_led_value(1)
            elif button_X == 1:
                self.publish_led_value(2)
            elif button_Y == 1:
                self.publish_led_value(3)

            # Handle servo control based on axes 6 and 7
            axis_6_value = msg.axes[6]
            axis_7_value = msg.axes[7]

            if axis_7_value == 1:
                self.servo_position = 45
            elif axis_7_value == -1:
                self.servo_position = 0
            elif axis_6_value == 1:
                self.servo_position = min(self.servo_position + 1, 45)
            elif axis_6_value == -1:
                self.servo_position = max(self.servo_position - 1, 0)

            # Publish the servo position
            servo_msg = Int8()
            servo_msg.data = self.servo_position
            self.servo_publisher.publish(servo_msg)

    def publish_led_value(self, value):
        leds_msg = Int8()
        leds_msg.data = value
        self.leds_publisher.publish(leds_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
