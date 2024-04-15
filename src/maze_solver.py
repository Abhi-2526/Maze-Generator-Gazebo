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


class MazeDataCollector:
    def __init__(self):
        rospy.init_node('maze_data_collector')
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.rate = rospy.Rate(10)  # 10 Hz

        self.dataset = []
        self.sensor_data = []
        self.decision_data = []
        self.path_data = []
        self.maze_size = (11, 12)
        self.goal_pos = (10, 11)

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.dataset.append([position.x, position.y, orientation.z, orientation.w])

    def laser_callback(self, msg):
        laser_ranges = msg.ranges
        if len(self.dataset) > 0:
            self.dataset[-1].extend(laser_ranges)
        else:
            self.dataset.append(laser_ranges)

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
        start_pos = (0, 0)
        open_list = []
        closed_list = set()

        start_node = Node(start_pos, 0, None)
        heapq.heappush(open_list, (start_node.cost + self.heuristic(start_pos), start_node))

        path = []
        while open_list:
            current_node = heapq.heappop(open_list)[1]
            closed_list.add(current_node.pos)

            if current_node.pos == self.goal_pos:
                while current_node:
                    path.append(current_node.pos)
                    current_node = current_node.parent
                path.reverse()
                path = path[1:]
                print("Path found:", path)
                break

            neighbors = self.get_neighbors(current_node.pos)
            for neighbor_pos in neighbors:
                if neighbor_pos in closed_list:
                    continue

                if laser_ranges[0] > 0.5:
                    new_cost = current_node.cost + 1
                    new_node = Node(neighbor_pos, new_cost, current_node)
                    heapq.heappush(open_list, (new_node.cost + self.heuristic(neighbor_pos), new_node))
                else:
                    print("Obstacle detected at", neighbor_pos)

        self.sensor_data.append(laser_ranges)

        twist = Twist()
        if path:
            next_pos = path[0]
            direction = (next_pos[0] - current_node.pos[0], next_pos[1] - current_node.pos[1])

            if direction[0] > 0 and laser_ranges[0] > 0.5:  # Moving right
                twist.linear.x = 0.2
                print("Moving right")
                self.decision_data.append("Moving right")
            elif direction[0] < 0 and laser_ranges[180] > 0.5:  # Moving left
                twist.linear.x = -0.2
                print("Moving left")
                self.decision_data.append("Moving left")
            elif direction[1] > 0 and laser_ranges[90] > 0.5:  # Moving up
                twist.linear.x = 0.2
                print("Moving up")
                self.decision_data.append("Moving up")
            elif direction[1] < 0 and laser_ranges[270] > 0.5:  # Moving down
                twist.linear.x = -0.2
                print("Moving down")
                self.decision_data.append("Moving down")
            else:
                print("Obstacle detected, not moving")
                self.decision_data.append("Obstacle detected, not moving")
        else:
            print("No path found")
            self.decision_data.append("No path found")

        if path:
            self.path_data.append(path[0])

        return twist

    def run(self):
        while not rospy.is_shutdown():
            if len(self.dataset) > 0 and len(self.dataset[-1]) >= 360:
                laser_ranges = self.dataset[-1][-360:]
                twist = self.a_star_maze_solver(laser_ranges)
                self.vel_pub.publish(twist)
                self.dataset[-1].extend([twist.linear.x, twist.linear.y, twist.angular.z])
            self.rate.sleep()

    def save_dataset(self):
        with open('sensor_data.csv', 'w') as file:
            writer = csv.writer(file)
            writer.writerows(self.sensor_data)

        with open('decision_data.csv', 'w') as file:
            writer = csv.writer(file)
            writer.writerow(self.decision_data)

        with open('path_data.csv', 'w') as file:
            writer = csv.writer(file)
            writer.writerows(self.path_data)


if __name__ == '__main__':
    worlds_dir = os.path.join(os.path.dirname(__file__), '..', 'worlds')

    maze_worlds = [file for file in os.listdir(maze_worlds_dir) if file.endswith('.world')]

    try:
        for world_file in maze_worlds:
            os.system(f"roslaunch Maze-Generator-Gazebo maze_world.launch world_file:={worlds_dir}/{world_file}")
            collector = MazeDataCollector()
            collector.run()
            collector.save_dataset()
            os.system("rosnode kill -a")

    except rospy.ROSInterruptException:
        pass
