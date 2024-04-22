#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Header, ColorRGBA

def publish_fixed_marker():
    # Initialize ROS node
    rospy.init_node('fixed_marker_publisher')

    # Create a publisher object
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    # Wait for the publisher to establish connection to subscribers
    rospy.sleep(1)

    # Define the fixed point (change x, y, z as needed)
    point = Point(x=1.0, y=2.0, z=0.0)

    # Create a Marker message
    marker = Marker()
    marker.header.frame_id = "map"  # Or your specific frame
    marker.header.stamp = rospy.Time.now()
    marker.ns = "fixed_point"
    marker.id = 0
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.pose.position = point
    marker.pose.orientation.w = 1.0  # No rotation
    marker.scale.x = 0.2  # Diameter of the sphere in meters
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)  # Green color
    marker.lifetime = rospy.Duration()  # 0 means forever

    # Loop to publish the marker at 1 Hz until the node is shutdown
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        marker_pub.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_fixed_marker()
    except rospy.ROSInterruptException:
        pass
