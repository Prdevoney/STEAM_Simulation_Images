# Clone: git clone https://github.com/ros2/teleop_twist_keyboard.git 
# Use this script instead of given teleop_twist_keyboard.py

import sys
import tty
import threading

import geometry_msgs.msg
import rclpy

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

msg = """
This node takes keypresses from the keyboard and publishes them
as Twist messages. It works best with a US keyboard layout.
---------------------------
Moving around:
        i
    j   k   l
        ,

i : move forward
, : move backward
j : turn left
l : turn right

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    ',': (-1, 0, 0, 0),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}

def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return f'currently:\tspeed {speed}\tturn {turn}'


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('teleop_twist_keyboard')

    pub = node.create_publisher(geometry_msgs.msg.Twist, '/model/tugbot/cmd_vel', 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    speed = 0.5
    turn = 1.0
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0

    twist_msg = geometry_msgs.msg.Twist()

    try:
        print(msg)
        print(vels(speed, turn))
        while True:
            key = getKey(settings)
            print(f"Key Pressed: {key}")  # Debugging: Print keypress
            if key in moveBindings.keys():
                x, y, z, th = moveBindings[key]
                print(f"Updated: x={x}, th={th}")
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed, turn))
            else:
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
                if (key == '\x03'):
                    break

            twist_msg.linear.x = x * speed
            twist_msg.angular.z = th * turn
            print(f"Publishing: linear.x={twist_msg.linear.x}, angular.z={twist_msg.angular.z}")
            pub.publish(twist_msg)

    except Exception as e:
        print(e)

    finally:
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        pub.publish(twist_msg)
        rclpy.shutdown()
        spinner.join()

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()