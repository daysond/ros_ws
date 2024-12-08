#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
from geometry_msgs.msg import Twist


def stop_robot(pub):
    """Publishes a stop command to the robot."""
    stop_msg = Twist()
    stop_msg.linear.x = 0.0
    stop_msg.linear.y = 0.0
    stop_msg.angular.z = 0.0
    pub.publish(stop_msg)
    rospy.sleep(1)

def move_in_direction(pub, linear_x, linear_y, angular_z, duration, with_stop=True):
    """Moves the robot in the specified direction."""
    move_msg = Twist()
    move_msg.linear.x = linear_x
    move_msg.linear.y = linear_y
    move_msg.angular.z = angular_z
    pub.publish(move_msg)
    rospy.sleep(duration)
    if with_stop:
        stop_robot(pub)

def move_square_simulation(pub, repeat_times):
    """Moves the robot in a 1m square pattern for simulation."""
    for _ in range(repeat_times):
        # (0, 0, 0°) to (1, 0, 0°) - face the direction of travel
        move_in_direction(pub, linear_x=0.1, linear_y=0.0, angular_z=0.0, duration=10)

        # (1, 0, 0°) to (1, 1, 0°) - face the outside of the square
        move_in_direction(pub, linear_x=0.0, linear_y=0.1, angular_z=0.0, duration=10)

        # (1, 1, -90°) to (0, 1, -90°) - rotate first to face the inside
        move_in_direction(pub, linear_x=0.0, linear_y=0.0, angular_z=-0.2, duration=7.5)
        move_in_direction(pub, linear_x=0.0, linear_y=-0.1, angular_z=0.0, duration=10)

        # (0, 1, -90°) to (0, 0, 0°) - rotate while traveling
        x = 0.1
        y = 0.0
        z = 0.01
        for _ in range(10):  # Gradually rotate while moving westward
            move_in_direction(pub, linear_x=x, linear_y=-y, angular_z=0.125, duration=1.25, with_stop=False)
            x = x - 1 * z
            y = y + 1 * z

        stop_robot(pub)  # Stop after completing the movement

def move_square_real(pub, repeat_times):
    """Moves the robot in a 1m square pattern for the real-world."""
    for _ in range(repeat_times):
        # (0, 0, 0°) to (1, 0, 0°)
        move_in_direction(pub, linear_x=0.05, linear_y=0.0, angular_z=0.0, duration=15)

        # (1, 0, 0°) to (1, 1, 0°)
        move_in_direction(pub, linear_x=0.0, linear_y=0.05, angular_z=0.0, duration=15)

        # (1, 1, -90°) to (0, 1, -90°)
        move_in_direction(pub, linear_x=0.0, linear_y=0.0, angular_z=-0.1, duration=10)
        move_in_direction(pub, linear_x=0.0, linear_y=-0.05, angular_z=0.0, duration=15)

        # (0, 1, -90°) to (0, 0, 0°)
        x = 0.05
        y = 0.0
        z = 0.005
        for _ in range(10):
            move_in_direction(pub, linear_x=x, linear_y=-y, angular_z=0.1, duration=1.5, with_stop=False)
            x = x - 1 * z
            y = y + 1 * z

        stop_robot(pub)  # Stop after completing the movement

def main():
    # Initialize ROS node
    rospy.init_node('jetauto_gazebo_controller', anonymous=True)

    # Publisher for cmd_vel
    pub = rospy.Publisher('/jetauto_controller/cmd_vel', Twist, queue_size=10)

    # Ask for the number of repetitions
    repeat_times = int(input("How many times should the square movement happen? "))

    # Ask for the environment type
    environment = input("real/gazebo: ").lower()

    # Wait for a start command
    input("Press Enter to start moving in a square pattern...")

    # Move in a square pattern depending on the environment
    if environment == "gazebo":
        move_square_simulation(pub, repeat_times)
    elif environment == "real":
        move_square_real(pub, repeat_times)
    else:
        print("Invalid environment. Only 'real' or 'gazebo' are available.")
        return

    # Stop the robot at the end
    stop_robot(pub)
    print("Movement completed.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

