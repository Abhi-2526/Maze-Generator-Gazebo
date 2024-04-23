import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import csv
import math

def log_data(writer, pose, laser_data, twist, target_distance):
    orientation = pose.pose.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    data = [
        pose.pose.pose.position.x, pose.pose.pose.position.y, yaw,
        laser_data.ranges[0], laser_data.ranges[90], laser_data.ranges[180], laser_data.ranges[270],
        twist.linear.x, twist.angular.z,
        target_distance
    ]
    writer.writerow(data)

def data_logger():
    rospy.init_node('data_logger')
    rospy.loginfo("Data logging node initialized.")

    # Set the target position (replace with your desired target)
    target_x = 11.0
    target_y = 11.0

    # Open a CSV file for logging data
    with open('imitation_learning_dataset.csv', 'w') as csvfile:
        fieldnames = ['pos_x', 'pos_y', 'yaw', 'laser_front', 'laser_left', 'laser_back', 'laser_right',
                      'linear_vel', 'angular_vel', 'target_distance']
        writer = csv.writer(csvfile)
        writer.writerow(fieldnames)

        rate = rospy.Rate(10)  # 10 Hz logging frequency
        while not rospy.is_shutdown():
            # Get the current pose of the robot
            pose = rospy.wait_for_message('/odom', Odometry)

            # Get the current laser scan data
            laser_data = rospy.wait_for_message('/scan', LaserScan)

            # Get the current velocity of the robot
            twist = rospy.wait_for_message('/cmd_vel', Twist)

            # Calculate the distance to the target position
            dx = target_x - pose.pose.pose.position.x
            dy = target_y - pose.pose.pose.position.y
            target_distance = math.sqrt(dx**2 + dy**2)

            # Log the data
            log_data(writer, pose, laser_data, twist, target_distance)

            rate.sleep()

if __name__ == '__main__':
    try:
        data_logger()
    except rospy.ROSInterruptException:
        pass

