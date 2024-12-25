#!/usr/bin/env python3

import rospy
import actionlib
import numpy as np
import math
import tf
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class GoalDetermination:
    def __init__(self):
        rospy.init_node('goal_determination')

        # MoveBase Action Client
        self.navclient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.navclient.wait_for_server()
        rospy.loginfo("Connected to move_base action server.")

        # Map Subscription
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        # Map Data
        self.map_data = None
        self.resolution = None
        self.origin = None

    def map_callback(self, data):
        try:
            # Reshape the map data into a 2D NumPy array
            self.map_data = np.array(data.data, dtype=np.int8).reshape((data.info.height, data.info.width))
            self.resolution = data.info.resolution
            self.origin = (data.info.origin.position.x, data.info.origin.position.y)
            unexplored_cells = np.count_nonzero(self.map_data == -1)

            # Log the number of unexplored cells
            rospy.loginfo(f"Unexplored cells: {unexplored_cells}")
            if unexplored_cells == 0:
                rospy.loginfo("Exploration complete: No unexplored areas remaining.")
                self.exploring = False
        except Exception as e:
            rospy.logerr(f"Error processing map: {e}")

    def get_robot_position(self):
        """
        Get the robot's current position using the tf tree.
        """
        try:
            listener = tf.TransformListener()
            listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(4.0))
            (trans, rot) = listener.lookupTransform("map", "base_link", rospy.Time(0))
            return trans[0], trans[1]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Unable to get robot position.")
            return None, None

    def find_nearest_unexplored(self):
        """
        Find the nearest unexplored area (-1 in the map data).
        """
        if self.map_data is None:
            rospy.logwarn("Map data is not available.")
            return None

        unexplored_indices = np.argwhere(self.map_data == -1)
        robot_x, robot_y = self.get_robot_position()
        if robot_x is None or robot_y is None:
            rospy.logwar
            return None

        nearest_goal = None
        min_distance = float('inf')

        for index in unexplored_indices:
            goal_x = self.origin[0] + index[1] * self.resolution
            goal_y = self.origin[1] + index[0] * self.resolution
            distance = ((goal_x - robot_x)**2 + (goal_y - robot_y)**2)**0.5

            if distance < min_distance and self.is_goal_valid(goal_x, goal_y):
                nearest_goal = (goal_x, goal_y)
                min_distance = distance

        return nearest_goal

    def is_goal_valid(self, x, y):
        """
        Check if the goal is valid (free space and sufficiently far from the robot).
        """
        goal_grid_x = int((x - self.origin[0]) / self.resolution)
        goal_grid_y = int((y - self.origin[1]) / self.resolution)

        if self.map_data[goal_grid_y, goal_grid_x] != 0:  # 0: Free space
            return False

        robot_x, robot_y = self.get_robot_position()
        if robot_x is None or robot_y is None:
            return False

        distance = ((x - robot_x)**2 + (y - robot_y)**2)**0.5
        if distance < 0.5:  # Too close to the robot
            return False

        return True

    def calculate_orientation(self, x1, y1, x2, y2):
        """
        Calculate the yaw angle for the robot to face the goal.
        """
        return math.atan2(y2 - y1, x2 - x1)

    def send_goal(self, x, y, yaw):
        """
        Send a navigation goal to move_base.
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)

        rospy.loginfo(f"Sending goal to ({x:.2f}, {y:.2f}) with orientation {yaw:.2f} rad.")
        self.navclient.send_goal(goal)
        finished = self.navclient.wait_for_result()

        if not finished:
            rospy.logerr("Action server not available!")
        else:
            rospy.loginfo("Goal reached or action completed.")

    def determine_and_send_goal(self):
        """
        Main function to determine and send a goal.
        """
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            goal = self.find_nearest_unexplored()
            if goal:
                x, y = goal
                robot_x, robot_y = self.get_robot_position()
                if robot_x is not None and robot_y is not None:
                    yaw = self.calculate_orientation(robot_x, robot_y, x, y)
                    self.send_goal(x, y, yaw)
            else:
                rospy.loginfo("No valid goals found or map is fully explored.")
                break
            rate.sleep()


if __name__ == "__main__":
    try:
        explorer = GoalDetermination()
        explorer.determine_and_send_goal()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exploration interrupted.")