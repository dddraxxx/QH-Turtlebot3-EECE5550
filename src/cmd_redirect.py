#!/usr/bin/env python3
import numpy as np
import math
import rospy
# --- Importing Message Types ---
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

############ CONFIGURATION VARIABLES ###################
# --- Command Publishers ---
command_publisher = None
# --- Specifications of the Robot ---
robot_width = 0.16  # width of robot chassis in meters
wheel_radius = 0.033  # radius of wheel in meters
# --- Timing Parameters ---
StartOfMovement = None
EndOfMovement = None
MovementPeriod = rospy.Duration(25.0)
SpinPeriod = rospy.Duration(15)
# --- Data from Sensors ---
lidar_data = None


def retrieve_lidar_data(message):
    """
    Retrieve data from LiDAR's laserscan.
     - Returns a 360-element list representing degrees.
    """
    global lidar_data
    lidar_data = message.ranges


def process_incoming_command(message):
    """
    Handle commands from the motion planner.
    Commands can be forwarded directly or modified as necessary.
    A spin is initiated after MovementPeriod has elapsed.
    Conditions: SpinPeriod * new_message.angular.z = 2 pi.
    """
    global StartOfMovement, EndOfMovement
    if rospy.Time.now() >= EndOfMovement:
        # Modify the command to initiate a spin.
        new_message = Twist()
        new_message.linear.x = 0
        new_message.angular.z = np.round(2 * math.pi / SpinPeriod.to_sec(), 2)
        StartOfSpin = rospy.Time.now()
        EndOfSpin = StartOfSpin + SpinPeriod
        # Spin in place for SpinPeriod resulting in a 360-degree turn
        while rospy.Time.now() <= EndOfSpin:
            # Executing a 360-degree in-place spin.
            print("Spinning to locate markers.")
            command_publisher.publish(new_message)
            rospy.sleep(0.1)
        # Prepare for the next spin.
        StartOfMovement = rospy.Time.now()
        EndOfMovement = StartOfMovement + MovementPeriod
    else:
        command_publisher.publish(message)


def main():

    global command_publisher, StartOfMovement, EndOfMovement

    rospy.init_node('robot_control')

    StartOfMovement = rospy.Time.now()
    EndOfMovement = StartOfMovement + MovementPeriod

    # Establish publisher for robot commands.
    command_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    # Listen to LiDAR for obstacle data.
    rospy.Subscriber('/scan', LaserScan, retrieve_lidar_data, queue_size=1)
    # Receive commands from the motion planning system.
    rospy.Subscriber('/cmd_redirect', Twist, process_incoming_command, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
