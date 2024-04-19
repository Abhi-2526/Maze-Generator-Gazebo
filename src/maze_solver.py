import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import sys, select, termios, tty
import csv
import math
import time

pub = None

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def log_data(writer, pose, laser_data, twist, target_distance, key_pressed, timestamp):
    orientation = pose.pose.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    data = [
        pose.pose.pose.position.x, pose.pose.pose.position.y, yaw,
        laser_data.ranges[0], laser_data.ranges[90], laser_data.ranges[180], laser_data.ranges[270],
        twist.linear.x, twist.angular.z,
        target_distance,
        key_pressed,
        timestamp
    ]
    writer.writerow(data)

def teleop():
    global pub
    rospy.init_node('turtlebot_teleop')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.loginfo("TurtleBot Teleoperation node initialized.")
    rospy.loginfo("Use arrow keys to move the TurtleBot:")
    rospy.loginfo("  Up arrow: Move forward")
    rospy.loginfo("  Down arrow: Move backward")
    rospy.loginfo("  Left arrow: Rotate counterclockwise")
    rospy.loginfo("  Right arrow: Rotate clockwise")
    rospy.loginfo("Press 'q' to quit.")

    # Set the target position (replace with your desired target)
    target_x = 11.0
    target_y = 11.0

    twist = Twist()

    # Open a CSV file for logging data
    with open('imitation_learning_dataset.csv', 'w') as csvfile:
        fieldnames = ['pos_x', 'pos_y', 'yaw', 'laser_front', 'laser_left', 'laser_back', 'laser_right',
                      'linear_vel', 'angular_vel', 'target_distance', 'key_pressed', 'timestamp']
        writer = csv.writer(csvfile)
        writer.writerow(fieldnames)

        while not rospy.is_shutdown():
            key = getKey()
            rospy.loginfo(f"Key: {key}")
            if key == 'w':  
                twist.linear.x = 0.2
                twist.angular.z = 0.0
            elif key == 's':  
                twist.linear.x = -0.2
                twist.angular.z = 0.0
            elif key == 'a': 
                twist.linear.x = 0.0
                twist.angular.z = 0.5
            elif key == 'd': 
                twist.linear.x = 0.0
                twist.angular.z = -0.5
            elif key == 'q':
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                rospy.loginfo("Quitting...")
                rospy.signal_shutdown("User requested shutdown.")
            else:
                key == "break"
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            rospy.loginfo(f"Publishing twist: {twist}")
            pub.publish(twist)
            
            # Get the current pose of the robot
            pose = rospy.wait_for_message('/odom', Odometry)

            # Get the current laser scan data
            laser_data = rospy.wait_for_message('/scan', LaserScan)

            # Calculate the distance to the target position
            dx = target_x - pose.pose.pose.position.x
            dy = target_y - pose.pose.pose.position.y
            target_distance = math.sqrt(dx**2 + dy**2)

            # Log the data
            timestamp = time.time()
            log_data(writer, pose, laser_data, twist, target_distance, key, timestamp)

            rospy.sleep(0.1)

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    try:
        teleop()
    except rospy.ROSInterruptException:
        pass
    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

