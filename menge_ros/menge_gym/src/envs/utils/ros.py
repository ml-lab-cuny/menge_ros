from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
# import roslaunch
import subprocess
from numpy import arccos


def pose2array(pose: Pose):
    """
    extract 2D position and orientation from ROS geometry_msgs/Pose message

    :param pose: ROS geometry_msgs/Pose message

    :return: list [x, y, omega] (2D position x,y + orientation omega)
    """
    # only 2D poses (position x,y + rotation around z)
    x = pose.position.x
    y = pose.position.y
    # orientation quaternion
    omega = 2 * arccos(pose.orientation.w)

    return [x, y, omega]


def obstacle2array(obs_pose: Pose):
    """
    extract 2D position from ROS geometry_msgs/Pose message

    :param obs_pose: ROS geometry_msgs/Pose message

    :return: list [x, y] (2D position x,y)
    """
    # only 2D point objects (x,y - no orientation)
    x = obs_pose.position.x
    y = obs_pose.position.y
    return [x, y]


def marker2array(marker: Marker):
    """
    extract 2D position, orientation and radius from ROS geometry_msgs/Pose message

    :param marker: ROS visualization_msgs/Marker message

    :return: list [x, y, q_w, q_z, r] (2D position x,y + orientation omega + radius r)
    """
    # only 2D poses (position x,y + rotation around z)
    x = marker.pose.position.x
    y = marker.pose.position.y
    # orientation quaternion
    omega = 2 * arccos(marker.pose.orientation.w)

    # radius
    r = marker.scale.x / 2
    return [x, y, omega, r]


def start_roslaunch_file(pkg: str, launchfile: str, launch_cli_args: dict = None) -> subprocess.Popen:
    """
    start a ROS launchfile with arguments

    :param pkg: ROS package where launchfile is specified
    :param launchfile: name of the launch file
    :param launch_cli_args: dict of additional command line arguments {arg:value}

    :return: process of the launched file
    """

    # make sure launchfile is provided with extension
    if not str(launchfile).endswith('.launch'):
        launchfile += '.launch'

    launch_expr = ["roslaunch", pkg, launchfile]

    if launch_cli_args:
        # resolve cli arguments
        for key in launch_cli_args:
            launch_expr.append(str(key) + ':=' + str(launch_cli_args[key]))

    # launch file

    return subprocess.Popen(launch_expr)
