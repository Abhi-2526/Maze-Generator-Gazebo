import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import os
import csv
import math
from tf.transformations import euler_from_quaternion

class MazeDataCollector:
    def __init__(self):
        rospy.init_node('maze_data_collector')
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.rate = rospy.Rate(10)  # 10 Hz

        self.dataset = []
        self.odom_data = []
        self.sensor_data = []
        self.decision_data = []
        self.path_data = []
        self.maze_size = (12, 12)
        self.start_pos = (0.5, 0.5)
        self.goal_pos = (11, 11)
        self.current_pos = self.start_pos
        self.visited = set()
        self.path_stack = []

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.odom_data.append([position.x, position.y, orientation.x, orientation.y, orientation.z, orientation.w])

    def laser_callback(self, msg):
        laser_ranges = msg.ranges
        if len(self.dataset) > 0 and isinstance(self.dataset[-1], list):
            self.dataset[-1].extend(laser_ranges)
        else:
            self.dataset.append(list(laser_ranges))

    def get_neighbors(self, pos):
        movements = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        neighbors = []

        for move in movements:
            new_pos = (pos[0] + move[0], pos[1] + move[1])
            if 0 <= new_pos[0] < self.maze_size[0] and 0 <= new_pos[1] < self.maze_size[1]:
                neighbors.append(new_pos)

        return neighbors

    def dfs_maze_solver(self, laser_ranges):
        rospy.loginfo(f"In DFS solver")

        if self.odom_data:
            self.current_pos = tuple(self.odom_data[-1][:2])

        if abs(self.current_pos[0] - self.goal_pos[0]) < 0.1 and abs(self.current_pos[1] - self.goal_pos[1]) < 0.1:
            rospy.loginfo("Goal reached!")
            return Twist()

        self.visited.add(self.current_pos)
        self.path_stack.append(self.current_pos)

        neighbors = self.get_neighbors(self.current_pos)
        rospy.loginfo(f"Current position: {self.current_pos}")
        rospy.loginfo(f"Neighbors: {neighbors}")

        index = 0
        neighbors = list(neighbors)  # Ensure neighbors is a list if not already
        
        while index < len(neighbors):
            neighbor_pos = neighbors[index]
            if neighbor_pos not in self.visited:
                direction = (neighbor_pos[0] - self.current_pos[0], neighbor_pos[1] - self.current_pos[1])
                twist = Twist()
                target_orientation = None
                rospy.loginfo(f"The Direction is: {direction}")
                
                if direction[0] > 0 and laser_ranges[0] > 0.5:
                    self.decision_data.append("Moving forward")
                    twist.linear.x = 0.1
                    rospy.loginfo("Moving forward")
                elif direction[0] < 0 and laser_ranges[180] > 0.5:
                    self.decision_data.append("Moving backward")
                    twist.linear.x = -0.1
                    rospy.loginfo("Moving backward")
                elif direction[1] > 0 and laser_ranges[90] > 0.5:
                    self.decision_data.append("Turning left")
                    target_orientation = math.pi / 2
                    twist.angular.z = 0.1
                    rospy.loginfo("Turning left")
                elif direction[1] < 0 and laser_ranges[270] > 0.5:
                    self.decision_data.append("Turning right")
                    target_orientation = -math.pi / 2
                    twist.angular.z = -0.1
                    rospy.loginfo("Turning right")

                success = False
                while not success:
                    val = 0
                    val = self.wait_for_bot(neighbor_pos, target_orientation)
                    self.vel_pub.publish(twist)
                    rospy.loginfo(f"Published velocity command: {twist}")
                    rospy.loginfo("Waiting to reach the target or re-evaluate conditions...")
                    rospy.sleep(0.5)  # Wait before trying again or breaking out based on additional conditions
                    if val != 0:
                        rospy.loginfo("wtf is going on")
                        success = True

                if val == 2:
                    index += 1  # Move to the next neighbor only on success

        stop_twist = Twist()
        self.vel_pub.publish(stop_twist)
        rospy.loginfo("Bot stopped")

        return twist

        # Backtrack if no unvisited neighbors
        self.path_stack.pop()
        if self.path_stack:
            backtrack_pos = self.path_stack[-1]
            direction = (backtrack_pos[0] - self.current_pos[0], backtrack_pos[1] - self.current_pos[1])
            twist = Twist()
            target_orientation = None

            if direction[0] > 0:
                self.decision_data.append("Moving forward (backtrack)")
                twist.linear.x = 0.1
                rospy.loginfo("Moving forward (backtrack)")
            elif direction[0] < 0:
                self.decision_data.append("Moving backward (backtrack)")
                twist.linear.x = -0.1
                rospy.loginfo("Moving backward (backtrack)")
            elif direction[1] > 0:
                self.decision_data.append("Turning left (backtrack)")
                target_orientation = math.pi / 2
                twist.angular.z = 0.1
                rospy.loginfo("Turning left (backtrack)")
            elif direction[1] < 0:
                self.decision_data.append("Turning right (backtrack)")
                target_orientation = -math.pi / 2
                twist.angular.z = -0.1
                rospy.loginfo("Turning right (backtrack)")

            while not self.wait_for_bot(backtrack_pos, target_orientation):
                self.vel_pub.publish(twist)
                rospy.loginfo(f"Published velocity command: {twist}")
                rospy.sleep(0.5)

            return twist

        rospy.logwarn("No path found")
        self.decision_data.append("No path found")
        return Twist()

    def wait_for_bot(self, target_pos, target_orientation=None, position_threshold=0.5, orientation_threshold=0.1):
        if self.odom_data:
            current_pos = tuple(self.odom_data[-1][:2])
            current_orientation = self.odom_data[-1][2:6]
            rospy.loginfo(f"Odom Data: {self.odom_data[-1]}")
            rospy.loginfo(f"Current orientation: {current_orientation}")
            orie = euler_from_quaternion(current_orientation)
            rospy.loginfo(f"Orie: {orie}")
            current_yaw = orie[2]
            rospy.loginfo(f"Current position: {current_pos}")
            rospy.loginfo(f"Target position: {target_pos}")
            rospy.loginfo(f"Current yaw: {current_yaw}")
            rospy.loginfo(f"Target orientation: {target_orientation}")

            position_reached = abs(current_pos[0] - target_pos[0]) < position_threshold and abs(current_pos[1] - target_pos[1]) < position_threshold
            orientation_reached = False

            if target_orientation is not None:
                orientation_diff = abs(current_yaw - target_orientation)
                orientation_reached = orientation_diff < orientation_threshold

                if orientation_reached:
                    # Stop the bot's rotation
                    stop_twist = Twist()
                    self.vel_pub.publish(stop_twist)
                    rospy.loginfo("Bot stopped rotating")
                    return 1

            if position_reached:
                rospy.loginfo(f"Bot reached target position: {target_pos} and orientation: {target_orientation}")
                rospy.signal_shutdown('Objective Reached')
                return 2

        return 0

    def run(self):
        rospy.loginfo(f"In run")
        self.timer = rospy.Timer(rospy.Duration(1), self.collect_data)
        rospy.spin()

    def collect_data(self, event):
        rospy.loginfo(f"In collect")
        timestamp = rospy.Time.now().to_sec()
        if len(self.dataset) > 0 and len(self.dataset[-1]) >= 360:
            laser_ranges = self.dataset[-1][-360:]
            twist = self.dfs_maze_solver(laser_ranges)
            self.dataset[-1].extend([twist.linear.x, twist.linear.y, twist.angular.z])
            self.dataset[-1].extend(self.odom_data[-1])
            self.dataset[-1].append(timestamp)
            self.path_data.append(self.current_pos)
            self.sensor_data.append(laser_ranges)

    def save_dataset(self):
        with open('sensor_data.csv', 'w') as file:
            writer = csv.writer(file)
            writer.writerow(['laser_range_' + str(i) for i in range(360)] + ['twist_linear_x', 'twist_linear_y', 'twist_angular_z', 'odom_x', 'odom_y', 'odom_z', 'odom_w', 'timestamp'])
            writer.writerows(self.dataset)

        with open('decision_data.csv', 'w') as file:
            writer = csv.writer(file)
            writer.writerow(['decision', 'timestamp'])
            writer.writerows(zip(self.decision_data, [data[-1] for data in self.dataset]))

        with open('path_data.csv', 'w') as file:
            writer = csv.writer(file)
            writer.writerow(['path_x', 'path_y', 'timestamp'])
            writer.writerows([[path[0], path[1], data[-1]] for path, data in zip(self.path_data, self.dataset)])

        with open('odom_data.csv', 'w') as file:
            writer = csv.writer(file)
            writer.writerow(['odom_x', 'odom_y', 'odom_z', 'odom_w', 'timestamp'])
            writer.writerows([[data[0], data[1], data[2], data[3], data[-1]] for data in self.dataset])


if __name__ == '__main__':
    try:
        collector = MazeDataCollector()

        g_start_sim_time = rospy.get_time()
        print('Started at {} seconds (sim time)'.format(g_start_sim_time))

        collector.run()
        collector.save_dataset()

        end_sim_time = rospy.get_time()
        print('Finished at {} seconds (sim time)'.format(end_sim_time))
        print('Ran for {} seconds'.format(end_sim_time - g_start_sim_time))

    except rospy.ROSInterruptException:
        pass