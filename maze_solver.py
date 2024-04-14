import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import csv
import os
import heapq

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
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.rate = rospy.Rate(10)  # 10 Hz

        self.bridge = CvBridge()
        self.dataset = []
        self.dataset_file = 'maze_dataset.csv'
        self.maze_size = (10, 10)  # Example maze size, adjust according to your maze dimensions
        self.goal_pos = (9, 9)  # Example goal position, adjust according to your maze layout

    def odom_callback(self, msg):
        # Get the current position and orientation from odometry
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.dataset.append([position.x, position.y, orientation.z, orientation.w])

    def laser_callback(self, msg):
        # Get the LIDAR scan readings
        laser_ranges = msg.ranges
        self.dataset[-1].extend(laser_ranges)

    def image_callback(self, msg):
        # Get the image data
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Preprocess the image if needed (e.g., resize, normalize)
        # Append the image data to the dataset
        self.dataset[-1].append(image)

    def heuristic(self, pos):
        # Manhattan distance heuristic
        return abs(pos[0] - self.goal_pos[0]) + abs(pos[1] - self.goal_pos[1])

    def get_neighbors(self, pos):
        # Define the possible movements (up, down, left, right)
        movements = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        neighbors = []

        for move in movements:
            new_pos = (pos[0] + move[0], pos[1] + move[1])
            if 0 <= new_pos[0] < self.maze_size[0] and 0 <= new_pos[1] < self.maze_size[1]:
                neighbors.append(new_pos)

        return neighbors

    def a_star_maze_solver(self, laser_ranges):
        # A* algorithm for maze solving
        start_pos = (0, 0)  # Example start position, adjust according to your maze layout
        open_list = []
        closed_list = set()

        start_node = Node(start_pos, 0, None)
        heapq.heappush(open_list, (start_node.cost + self.heuristic(start_pos), start_node))

        while open_list:
            current_node = heapq.heappop(open_list)[1]
            closed_list.add(current_node.pos)

            if current_node.pos == self.goal_pos:
                # Goal reached, backtrack to get the path
                path = []
                while current_node:
                    path.append(current_node.pos)
                    current_node = current_node.parent
                path.reverse()
                print("Path found:", path)
                break

            neighbors = self.get_neighbors(current_node.pos)
            for neighbor_pos in neighbors:
                if neighbor_pos in closed_list:
                    continue

                # Check if the neighbor is an obstacle based on laser range data
                if laser_ranges[0] > 0.5:  # Example obstacle distance threshold
                    new_cost = current_node.cost + 1
                    new_node = Node(neighbor_pos, new_cost, current_node)
                    heapq.heappush(open_list, (new_node.cost + self.heuristic(neighbor_pos), new_node))
                else:
                    print("Obstacle detected at", neighbor_pos)

        twist = Twist()
        if path:
            next_pos = path[1]  # Get the next position in the path
            if next_pos[0] > current_node.pos[0]:
                twist.linear.x = 0.2  # Move right
                print("Moving right")
            elif next_pos[0] < current_node.pos[0]:
                twist.linear.x = -0.2  # Move left
                print("Moving left")
            elif next_pos[1] > current_node.pos[1]:
                twist.linear.y = 0.2  # Move up
                print("Moving up")
            elif next_pos[1] < current_node.pos[1]:
                twist.linear.y = -0.2  # Move down
                print("Moving down")
        else:
            print("No path found")

        return twist

    def run(self):
        while not rospy.is_shutdown():
            # Get the latest laser scan readings
            laser_ranges = self.dataset[-1][-360:]  # Assumes 360 laser scan readings

            # Call the A* maze solver algorithm to get the velocity command
            twist = self.a_star_maze_solver(laser_ranges)
            self.vel_pub.publish(twist)

            # Store the actions (velocity commands) in the dataset
            self.dataset[-1].extend([twist.linear.x, twist.linear.y, twist.angular.z])

            self.rate.sleep()

    def save_dataset(self):
        with open(self.dataset_file, 'w') as file:
            writer = csv.writer(file)
            writer.writerows(self.dataset)


if __name__ == '__main__':
    # Specify the directory containing the maze world files
    maze_worlds_dir = '/path/to/maze/worlds/'

    # Get the list of maze world files
    maze_worlds = [file for file in os.listdir(maze_worlds_dir) if file.endswith('.world')]

    try:
        for world_file in maze_worlds:
            # Launch the Gazebo world
            os.system(f"roslaunch turtlebot3_gazebo turtlebot3_world.launch world_file:={maze_worlds_dir}/{world_file}")

            # Create an instance of MazeDataCollector
            collector = MazeDataCollector()
            collector.run()

            # Save the dataset for the current maze world
            collector.save_dataset()

            # Shutdown the Gazebo simulation
            os.system("rosnode kill -a")

    except rospy.ROSInterruptException:
        pass