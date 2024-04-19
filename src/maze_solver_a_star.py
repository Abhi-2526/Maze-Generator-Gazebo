import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import os
import heapq
import csv

class Node:
    def __init__(self, pos, cost, parent):
        self.pos = pos
        self.cost = cost
        self.parent = parent

    def __lt__(self, other):
        return self.cost < other.cost


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
        self.goal_pos = (2.5, 2.5)

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.odom_data.append([position.x, position.y, orientation.z, orientation.w])

    def laser_callback(self, msg):
        laser_ranges = msg.ranges
        self.dataset.append(list(laser_ranges))

    def heuristic(self, pos):
        return abs(pos[0] - self.goal_pos[0]) + abs(pos[1] - self.goal_pos[1])

    def get_neighbors(self, pos):
        movements = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        neighbors = []

        for move in movements:
            new_pos = (pos[0] + move[0], pos[1] + move[1])
            if 0 <= new_pos[0] < self.maze_size[0] and 0 <= new_pos[1] < self.maze_size[1]:
                neighbors.append(new_pos)

        return neighbors

    def a_star_maze_solver(self, laser_ranges):
        rospy.loginfo(f"In A* solver")
        # if len(self.odom_data) > 0:
        #     current_pos = tuple(self.odom_data[-1][:2])
        # else:
        current_pos = (0.5, 0.5)

        open_list = []
        closed_list = set()

        start_node = Node(current_pos, 0, None)
        heapq.heappush(open_list, (start_node.cost + self.heuristic(current_pos), start_node))
        rospy.loginfo(f"Proceeding in A*")
        path = []
        while open_list:
            rospy.loginfo(f"Inside while")
            current_node = heapq.heappop(open_list)[1]
            rospy.loginfo(f"Current node: {current_node.pos}")
            closed_list.add(current_node.pos)

            goal_threshold = 0.1
            if abs(current_node.pos[0] - self.goal_pos[0]) < goal_threshold and abs(current_node.pos[1] - self.goal_pos[1]) < goal_threshold:
                while current_node:
                    path.append(current_node.pos)
                    current_node = current_node.parent
                path.reverse()
                path = path[1:]
                rospy.loginfo(f"Path found: {path}")
                break

            neighbors = self.get_neighbors(current_node.pos)
            rospy.loginfo(f"Neighbors: {neighbors}")

            for neighbor_pos in neighbors:
                if neighbor_pos in closed_list:
                    continue

                neighbor_direction = (neighbor_pos[0] - current_node.pos[0], neighbor_pos[1] - current_node.pos[1])
                if neighbor_direction[0] > 0:
                    laser_index = 0
                elif neighbor_direction[0] < 0:
                    laser_index = 180
                elif neighbor_direction[1] > 0:
                    laser_index = 90
                else:
                    laser_index = 270

                if laser_ranges[laser_index] < 0.5:
                    rospy.logwarn(f"Obstacle detected between {neighbor_pos} and {current_node.pos}")
                    continue

                new_cost = current_node.cost + 1
                new_node = Node(neighbor_pos, new_cost, current_node)
                heapq.heappush(open_list, (new_node.cost + self.heuristic(neighbor_pos), new_node))

        rospy.loginfo(f"More progress made in A*")
        self.sensor_data.append(laser_ranges)

        twist = Twist()
        if path:
            print(f"path found: {path}")
            next_pos = path[0]
            if current_node is not None:
                direction = (next_pos[0] - current_node.pos[0], next_pos[1] - current_node.pos[1])
            else:
                direction = (next_pos[0] - current_pos[0], next_pos[1] - current_pos[1])

            if direction[0] != 0 and laser_ranges[0] > 0.5:
                twist.linear.x = 0.2
                rospy.loginfo("Moving forward")
                self.decision_data.append("Moving forward")
            elif direction[1] != 0:
                if laser_ranges[90] > 0.5:
                    twist.angular.z = 1.57
                    rospy.loginfo("Rotating 90 degrees clockwise")
                    self.decision_data.append("Rotating 90 degrees clockwise")
                elif laser_ranges[270] > 0.5:
                    twist.angular.z = -1.57
                    rospy.loginfo("Rotating 90 degrees counterclockwise")
                    self.decision_data.append("Rotating 90 degrees counterclockwise")
                else:
                    rospy.logwarn("Obstacle detected, not moving")
                    self.decision_data.append("Obstacle detected, not moving")
            else:
                rospy.logwarn("Obstacle detected, not moving")
                self.decision_data.append("Obstacle detected, not moving")

            self.vel_pub.publish(twist)
            rospy.loginfo(f"Published velocity command: {twist}")
            rospy.sleep(0.5)
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.vel_pub.publish(twist)

        if path:
            self.path_data.append(path[0])

        return twist

    def run(self):
        rospy.loginfo(f"In run")
        self.timer = rospy.Timer(rospy.Duration(1), self.collect_data)
        rospy.spin()

    def collect_data(self, event):
        rospy.loginfo(f"In collect")
        timestamp = rospy.Time.now().to_sec()
        if len(self.dataset) > 0:
            laser_ranges = self.dataset[-1]
            twist = self.a_star_maze_solver(laser_ranges)
            self.dataset[-1].extend([twist.linear.x, twist.linear.y, twist.angular.z])
            self.dataset[-1].extend(self.odom_data[-1])
            self.dataset[-1].append(timestamp)

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
