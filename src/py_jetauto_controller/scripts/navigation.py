#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_srvs.srv import Empty

class Navigator:

    def __init__(self):
        """Initialize the navigation system."""
        # rospy.init_node("navigate_to_pickup_and_dropoff", anonymous=True)

        # Set the initial pose
        self.set_initial_pose()

        # Create an action client for the move_base action server
        self.client = actionlib.SimpleActionClient("/jetauto_1/move_base", MoveBaseAction)
        rospy.loginfo("Waiting for the move_base action server to come up...")
        self.client.wait_for_server()

        # Clear costmaps
        self.clear_costmaps()

    def set_initial_pose(self):
        """Prompt user for the initial pose and publish it."""
        rospy.loginfo("Setting initial pose of the robot.")
        pub = rospy.Publisher(
            "/jetauto_1/initialpose", PoseWithCovarianceStamped, queue_size=10
        )
        rospy.sleep(1)  # Allow time for the publisher to register

        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "jetauto_1/map"
        pose.header.stamp = rospy.Time.now()

        rospy.loginfo(
            """
            Orientation Values for Common Angles:
            - Forward: z=0.0, w=1.0
            - Backward: z=1.0, w=0.0
            - Left (90° CCW): z=0.707, w=0.707
            - Right (90° CW): z=-0.707, w=0.707
            """
        )


         ## !! Change these based on different maps !! ##
        # pick up init pose
        x = -0.277
        y = 0.024
        z = 0.0
        w = 1.0

         ## !! Change these based on different maps !! ##
        # drop off init pose
        # x = 3.02
        # y = 1.76
        # z = 0.0
        # w = 1.0

        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        pose.pose.pose.orientation.z = z
        pose.pose.pose.orientation.w = w

        rospy.loginfo("Publishing initial pose...")
        for _ in range(10):  # Publish multiple times to ensure it gets registered
            pub.publish(pose)
            rospy.sleep(0.1)

        return pose  # Return the initial pose for reuse later

    def rotate_robot(self):
        """Rotate the robot in place to help AMCL converge or as part of goal behavior."""
        rospy.loginfo("Rotating the robot in place...")
        pub = rospy.Publisher("/jetauto_1/cmd_vel", Twist, queue_size=10)
        twist = Twist()
        twist.angular.z = 2.0  # Increased rotation speed
        rate = rospy.Rate(10)  # 10 Hz
        for _ in range(50):  # Rotate for 5 seconds (10 Hz * 5 seconds)
            pub.publish(twist)
            rate.sleep()
        twist.angular.z = 0.0  # Stop rotation
        pub.publish(twist)
        rospy.loginfo("Rotation complete.")

    def clear_costmaps(self):
        """Clear the move_base costmaps."""
        rospy.loginfo("Clearing costmaps...")
        rospy.wait_for_service("/jetauto_1/move_base/clear_costmaps")
        try:
            clear_costmaps = rospy.ServiceProxy(
                "/jetauto_1/move_base/clear_costmaps", Empty
            )
            clear_costmaps()
            rospy.loginfo("Costmaps cleared.")
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call /move_base/clear_costmaps service: {}".format(e))

    def send_goal(self, x, y, z, w, rotate=True):
        """Send a navigation goal to move_base."""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "jetauto_1/map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = z
        goal.target_pose.pose.orientation.w = w

        rospy.loginfo("Sending goal: x={}, y={}, z={}, w={}".format(x, y, z, w))
        self.client.send_goal(goal)
        self.client.wait_for_result()

        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Successfully reached goal: x={}, y={}".format(x, y))
            if rotate:
                self.rotate_robot()  # Rotate after reaching the goal
        else:
            rospy.logwarn("Failed to reach goal: x={}, y={}".format(x, y))

    def navigate_to_checkpoints_for_drop_off(self):
        """Navigate through a series of intermediate checkpoints."""
        rospy.loginfo("Navigating to intermediate checkpoints...")

        ## !! Change these based on different maps !! ##
        checkpoints = [
            [2.71, 1.58, 0.0, -1.0],
            [2.56, 0.946, 0.0, -1.0],
            [1.12, 0.802, 0.0, 1.0],
            # [1.84, 1.74, 0.0, 1.0]
        ]
   

        for checkpoint in checkpoints:
            self.send_goal(checkpoint[0], checkpoint[1], checkpoint[2], checkpoint[3], rotate=False)
        rospy.loginfo("Finished navigating to all checkpoints.")

    def navigate_to_checkpoints_for_pickup(self):
        """Navigate through a series of intermediate checkpoints."""
        rospy.loginfo("Navigating to intermediate checkpoints...")

        ## !! Change these based on different maps !! ##
        checkpoints = [
            [1.02, 0.757, 0.0, -0.707],
            [1.04, 1.74, 0.0, 1.0],
            [1.84, 1.74, 0.0, 1.0]
        ]

        for checkpoint in checkpoints:
            self.send_goal(checkpoint[0], checkpoint[1], checkpoint[2], checkpoint[3], rotate=False)
        rospy.loginfo("Finished navigating to all checkpoints.")

    def navigate_to_pickup(self):
        """Navigate the robot to the pickup location."""
        rospy.loginfo("Navigating to the pickup point...")
         ## !! Change these based on different maps !! ##
        pickup_point = [2.80, 1.75, 0.0, 1.0]  # Pickup point
        self.send_goal(
            pickup_point[0], pickup_point[1], pickup_point[2], pickup_point[3], rotate=False
        )

    def navigate_to_dropoff(self):
        """Navigate the robot to the dropoff location."""
        rospy.loginfo("Navigating to the dropoff point...")
         ## !! Change these based on different maps !! ##
        dropoff_point = [0.439, -0.293, 0.0, 1.0]  # Dropoff point
        self.send_goal(
            dropoff_point[0], dropoff_point[1], dropoff_point[2], dropoff_point[3], rotate=False
        )

# def navigate():
#     """Main navigation function."""
#     nav = Navigator()
#     nav.navigate_to_checkpoints()
#     nav.navigate_to_pickup()
#     rospy.loginfo("Sleeping for 10 seconds...")
#     time.sleep(10)
#     nav.navigate_to_dropoff()
#     rospy.loginfo("Navigation through checkpoints, pickup, and dropoff completed!")

# if __name__ == "__main__":
#     try:
#         navigate()
#     except rospy.ROSInterruptException:
#         rospy.logerr("Navigation interrupted.")
