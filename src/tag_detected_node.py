#!/usr/bin/env python3
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
import tf2_ros
# --- Functions ---
import numpy as np
from datetime import datetime
from scipy.spatial.transform import Rotation as R

# --- Robot characteristics ---
w = 0.16 # chassis width (m).
r = 0.033 # wheel radius (m).
# --- TF Frames ---
TF_ORI = 'map'
TF_CAM = 'raspicam'
# --- Transforms ---
tf_listener = None
tf_buffer = None
T_CO = None # tf between cam and origin.
filepath = None # file where tags will be saved.
DT = 1 # period of timer that gets robot transform T_BO.
tags = {} # dictionary of tag poses, keyed by ID.


class KalmanFilter:
    def __init__(self, initial_state, process_noise, measurement_noise, initial_covariance):
        self.state = np.array(initial_state)
        self.P = initial_covariance
        self.Q = process_noise
        self.R = measurement_noise
        self.I = np.eye(len(initial_state))  # Identity matrix

    def predict(self):
        # Since the tags are static, prediction step only updates the state covariance
        self.P += self.Q

    def update(self, measurement):
        # Measurement update
        y = np.array(measurement) - self.state  # Measurement residual
        S = self.P + self.R  # Residual covariance
        K = self.P @ np.linalg.inv(S)  # Kalman Gain
        self.state += K @ y  # Update the state estimate
        self.P = (self.I - K) @ self.P  # Update covariance

    @property
    def T(self):
        q = self.state[3:]
        offset = self.state[:3]
        r = R.from_quat(q).as_matrix()
        last_row = np.array([0, 0, 0, 1])
        return np.vstack((np.hstack((r, offset.T)), last_row))

def r2state(T):
    rot_mat = T[:3,:3]
    rot_q = R.from_matrix(rot_mat).as_quat()
    print("rot_q: {}".format(rot_q))
    offset = T[:3,3]
    return np.concatenate((offset, rot_q))

def get_tag(tag_msg):
    global tags  # Dictionary to store KalmanFilter objects

    if len(tag_msg.detections) == 0:
        return

    for detection in tag_msg.detections:
        tag_id = detection.id
        tag_pose = detection.pose.pose.pose
        t = [tag_pose.position.x, tag_pose.position.y, tag_pose.position.z]

        q = [tag_pose.orientation.w, tag_pose.orientation.x, tag_pose.orientation.y, tag_pose.orientation.z]
        # make it into an affine matrix.
        r = R.from_quat(q).as_matrix()
        # make affine matrix for transformation.
        T_AC = np.array([[r[0][0],r[0][1],r[0][2],t[0]],
                        [r[1][0],r[1][1],r[1][2],t[1]],
                        [r[2][0],r[2][1],r[2][2],t[2]],
                        [0,0,0,1]])

        # we now have pose of tag in cam frame.

        # calculate global pose of the tag, unless the TF failed to be setup.
        if T_CO is None:
            print("Found tag, but cannot create global transform.")
            return
        # print("T_CO\n{}".format(T_CO))
        T_AO =T_CO@T_AC
        # print("The global tag pose is \n{}".format(T_AO))

        state = r2state(T_AO)
        if tag_id in tags:
            # Update existing tag
            tags[tag_id].predict()  # Predict the current state before the update
            tags[tag_id].update(state)  # Update with new measurement
            # tags[tag_id].state = state
            # print('UPDATING TAG: ', tag_id, 'STATE:', tags[tag_id].state)
        else:
            # Initialize a new Kalman Filter for each new tag
            initial_state = state
            initial_covariance = np.eye(7) * 0.1  # Initial uncertainty
            process_noise = np.eye(7) * 0.01  # Small since tags are static
            measurement_noise = np.eye(7) * 0.02  # Adjust based on expected noise level
            tags[tag_id] = KalmanFilter(initial_state, process_noise, measurement_noise, initial_covariance)
            # print('FOUND NEW TAG: ', tag_id, 'STATE:', tags[tag_id].state)

        # For debugging
        print('TAG:', tag_id, 'STATE:', tags[tag_id].state)

def get_trans(TF_TO, TF_FROM):
    global tf_buffer
    """
    Get the expected transform from tf.
    Use translation and quaternion from tf to construct a pose in SE(3).
    """
    try:
        # get most recent relative pose from the tf service.
        pose = tf_buffer.lookup_transform(TF_TO, TF_FROM,rospy.Time(0), rospy.Duration(4))
    except Exception as e:
        # requested transform was not found.
        print("Transform not found.")
        print("Exception: ", e)
        return None

    # extract translation and quaternion from tf pose.
    transformT = [pose.transform.translation.x, pose.transform.translation.y, pose.transform.translation.z]
    transformQ = (
        pose.transform.rotation.x,
        pose.transform.rotation.y,
        pose.transform.rotation.z,
        pose.transform.rotation.w)
    # get equiv rotation matrix from quaternion.
    r = R.from_quat(transformQ).as_matrix()

    # make affine matrix for transformation.
    return np.array([[r[0][0],r[0][1],r[0][2],transformT[0]],
                    [r[1][0],r[1][1],r[1][2],transformT[1]],
                    [r[2][0],r[2][1],r[2][2],transformT[2]],
                    [0,0,0,1]])


def dur_callback(event):
    """
    Update T_CO with newest pose from Cartographer.
    Save tags to file.
    """
    global T_CO
    T_CO = get_trans(TF_FROM=TF_CAM, TF_TO=TF_ORI)
    # save to 2 decimal places.
    # T_CO = np.round(T_CO, 2)
    # print("T_CO: ", T_CO)
    # print("T_base2map", get_transform(TF_TO='map', TF_FROM='base_link').round(2))
    # publish_tag_marker(pose=r2state(T_CO))
    # publish_tag_marker(tag_id=(2,), pose=r2state(get_transform(TF_TO='map', TF_FROM='base_link')))

    save_tags(tags)
    for tag_id in tags.keys():
        publish_tag_marker(tag_id)


def save_tags(tags):
    """
    We created a file whose name is the current time when the node is launched.
    All tags and IDs are saved to this file.
    This will recreate the file every timestep, so when the node ends,
        the file should reflect the most updated set of tag poses.
    """
    if not tags:
        # don't create the file if there aren't any tags detected yet.
        return
    data_for_file = []
    for id in tags.keys():
        print(id, tags[id]) # print to console for debugging.
        data_for_file.append("id: " + str(id))
        data_for_file.append("pose(x,y,z,quaternion): " + str(tags[id].state))
    np.savetxt(filepath, data_for_file, fmt="%s", delimiter=",")

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Header, ColorRGBA

def publish_tag_marker(tag_id=None, pose=None, frame_id='map'):
    marker = Marker()
    tag_id = (-1,) if tag_id is None else tag_id
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = f"apriltags_{tag_id[0]}"
    marker.id = int(tag_id[0])
    marker.type = marker.CUBE
    marker.action = marker.ADD
    pose = tags[tag_id].state if pose is None else pose
    marker.pose.position = Point(x=pose[0], y=pose[1], z=pose[2])
    marker.pose.orientation.x ,marker.pose.orientation.y ,marker.pose.orientation.z ,marker.pose.orientation.w =\
        pose[3:]
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Red color
    marker.lifetime = rospy.Duration()

    print("Publishing marker for tag: ", tag_id[0])
    global marker_publisher
    marker_publisher.publish(marker)

def main():
    global transform_listener, transform_buffer, detection_filepath, marker_publisher
    rospy.init_node('tag_detect')

    # Construct the filepath for storing detected tags.
    current_time = datetime.now()
    session_id = current_time.strftime("%Y-%m-%d-%H-%M-%S")
    detection_filepath = "detected-tags:" + str(session_id) + ".txt"

    # Initialize the Transform Buffer service.
    transform_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(1))
    transform_listener = tf2_ros.TransformListener(transform_buffer)

    # Listen for AprilTag detections.
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, get_tag, queue_size=1)
    # Initialize the publisher for tag visualization markers.
    marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    rospy.Timer(rospy.Duration(DT), dur_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
