#!/usr/bin/env python3

import sys
import threading
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import termios
import tty

msg = """
---------------------------
Drive around with WASD:
        w
   a    s    d

Adjust speed:
q : Increase linear speed
z : Decrease linear speed
e : Increase angular speed
c : Decrease angular speed

Move the blade with Up/Down arrows:
Up Arrow: Counter-clockwise
Down Arrow: Clockwise

CTRL-C to quit
"""

moveBindings = {
    "w": (1, 0, 0, 0),
    "s": (-1, 0, 0, 0),
    "a": (0, 0, 0, 1),
    "d": (0, 0, 0, -1),
}

speedBindings = {
    "q": (1.1, 1.0),
    "e": (1.0, 1.1),
    "z": (0.9, 1.0),
    "c": (1.0, 0.9),
}

bladeBindings = {
    "\x1b[A": -0.025,
    "\x1b[B": 0.025,
}


def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    if key == "\x1b":
        key += sys.stdin.read(2)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return "Current:\tlinear %s\tangular %s " % (speed, turn)


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node("keyboard_teleop")

    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    blade_pub = node.create_publisher(
        Float64MultiArray, "/position_controller/commands", 10
    )

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    speed = 0.25
    turn = 1.0
    blade_position = 0.0
    max_blade_position = 0.75
    min_blade_position = -0.75
    x = 0.0
    th = 0.0
    status = 0.0

    try:
        print(msg)
        print(vels(speed, turn))
        while True:
            key = getKey(settings)

            if key in moveBindings:
                x = moveBindings[key][0]
                th = moveBindings[key][3]
                twist = Twist()
                twist.linear.x = x * speed
                twist.angular.z = th * turn
                pub.publish(twist)
            elif key in speedBindings:
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                print(vels(speed, turn))
                if status == 14:
                    print(msg)
                status = (status + 1) % 15
            elif key in bladeBindings:
                blade_position = max(
                    min(blade_position + bladeBindings[key], max_blade_position),
                    min_blade_position,
                )
                blade_msg = Float64MultiArray()
                blade_msg.data = [blade_position]
                blade_pub.publish(blade_msg)
                continue
            else:
                x = 0.0
                th = 0.0
                if key == "\x03":
                    break

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        restoreTerminalSettings(settings)
        rclpy.shutdown()
        spinner.join()


if __name__ == "__main__":
    main()
