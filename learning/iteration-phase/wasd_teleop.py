#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty, threading, time

msg = """
Control Your AMR Using WASD!
---------------------------
Moving around:
        w
   a    s    d

anything else : stop / hold position
CTRL-C to quit
"""

# Layout mapping: 'key': (linear_x, angular_z)
moveBindings = {
    'w': (0.1, 0.0),   # Forward
    's': (-0.1, 0.0),  # Backward
    'a': (0.0, 1.0),   # Turn Left
    'd': (0.0, -1.0),  # Turn Right
}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = Node('wasd_teleop')
    pub = node.create_publisher(Twist, 'cmd_vel', 10)

    twist = Twist()
    
    # Secure background thread for continuous 50Hz publishing
    def publish_loop():
        while rclpy.ok():
            pub.publish(twist)
            time.sleep(0.02)

    pub_thread = threading.Thread(target=publish_loop, daemon=True)
    pub_thread.start()

    try:
        print(msg)
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                # Correctly mapping the 2-element tuple
                twist.linear.x = moveBindings[key][0]
                twist.angular.z = moveBindings[key][1]
            elif key == '\x03': # CTRL+C
                break
            elif key != '':
                # Stop if any unmapped key is pressed
                twist.linear.x = 0.0
                twist.angular.z = 0.0

    except Exception as e:
        print(e)
    finally:
        # Emergency stop on exit
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

