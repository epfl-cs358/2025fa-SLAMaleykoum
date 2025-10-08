#!/usr/bin/env python3
import sys
import tty
import termios
import select
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Int32, Bool

obstacle = False

def obstacle_callback(msg):
        global obstacle
        obstacle = msg.data

class Teleop(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        # Publishers
        self.motor_pub = self.create_publisher(Int8,  'motor_cmd',       10)
        self.steer_pub = self.create_publisher(Int32, 'servo_dir/angle', 10)

        #Subscriber
        self.create_subscription(Bool, '/emergency_stop', obstacle_callback,10)

        # Teleop state
        self.drive_power = 1   # –1 … +1 mapped to Int8
        self.angle       = 90  # starting at center (90 degrees)
        self.step_deg    = 5
        self.min_angle   = 60  # ±30° from center
        self.max_angle   = 120

        # Terminal setup
        self.fd = sys.stdin.fileno()
        self.orig_attrs = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

        self.get_logger().info(
            "Use W to go forward, S to go backward, space to stop, "
            "A/D to steer, Q to quit."
        )

    def restore_terminal(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.orig_attrs)

    def publish_drive(self, val: float):
        # Convert to Int8; e.g. ±1 → ±1
        msg = Int8()
        msg.data = int(val)
        self.motor_pub.publish(msg)

    def publish_steer(self):
        msg = Int32()
        msg.data = self.angle
        self.steer_pub.publish(msg)

    def run(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)

                if select.select([sys.stdin], [], [], 0)[0]:
                    c = sys.stdin.read(1).lower()
                    if c == 'q':
                        break
                    elif c == 'w':
                        # start forward motion
                        if obstacle:
                            self.publish_drive(0)
                        else :    
                            self.publish_drive(self.drive_power)
                    elif c == 's':
                        # start backward motion
                        self.publish_drive(-self.drive_power)
                    elif c == ' ':
                        # explicit stop
                        self.publish_drive(0)
                    elif c == 'a':
                        # steer left
                        self.angle = max(self.min_angle, self.angle - self.step_deg)
                        self.publish_steer()
                    elif c == 'd':
                        # steer right
                        self.angle = min(self.max_angle, self.angle + self.step_deg)
                        self.publish_steer()

        finally:
            self.restore_terminal()

def main():
    rclpy.init()
    node = Teleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
