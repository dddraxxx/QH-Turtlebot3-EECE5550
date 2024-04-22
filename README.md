# Team Qihua's Work for EECE5550

## To run our Code
Please install the necessary package, git clone our code in the catkin workspace of the turtlebot and the host PC and build the workspace. The following steps are to be followed to run our code:

1. In `turtlebot3\cartographer.launch` in `turtlebot3\slam` package, Add this to remap from `/map` topic to `/cmap`.
    * `<remap from="/map" to="/cmap" />`
    * From [github issue](https://github.com/hrnr/m-explore/issues/28#issuecomment-923616813)
    * Check `remapping.py`
2.  In `move_base.launch` in the `turtlebot3_navigation` package, rewrite arg name `/cmd_topic` as `/cmd_redirect` (originally `cmd_vel`).
3. On the host PC:
    * `roscore`
    * `rosrun tf static_transform_publisher 0.03 0 0.1 0 1.57 0 base_link raspicam 100`
    * `rosrun qihua tag_detected_node.py`
      * The node will detect the tags, and visualize the tags as a **red cube marker** in Rviz.
      * It use Kalman Filtering to update the tag pose estimation.
    * `roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=cartographer`
    * `rosrun qihua remapping.py`
      * Solve the remapping issue in the `move_base.launch` file.
    * `rosrun qihua cmd_redirect.py`
      * The node will interrupt the command, add the custom spin command for tag detection, and redirect it to the robot.
    * `roslaunch qihua explore_lite.launch`
      * Using explore-lite for frontier exploration.

4. Run the two raspicam commands on the robot via SSH.
    * `roslaunch raspicam_node camerav2_1280x960_10fps.launch enable_raw:=true`
    * `roslaunch qihua apriltag_detect.launch`
    * `roslaunch turtlebot3_bringup turtlebot3_robot.launch`


5. Saving result:
    * For the map:
      * `rosrun map_server map_saver -f <FILEPATH>`
    * For the tag location, it is auto-saved in :
      * `detected-tags:DATETIME.txt`.

---

**Credit**: This project was developed by Team Qihua in EECE5550, but the setup of the turtlebot and use of the library referred to [the great Bebop's project](https://github.com/kevin-robb/bebop-eece5550). The motion planning algorithm and the apriltag estimation are different and are original from Team Qihua.
