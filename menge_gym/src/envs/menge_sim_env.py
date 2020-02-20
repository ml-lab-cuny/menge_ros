#!/usr/bin/env python3

import gym
from gym import spaces
import numpy as np
from os import path
import rospy as rp
import rosnode
from geometry_msgs.msg import PoseArray, PoseStamped, Twist
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Bool
from menge_core.srv import RunSim
from .utils.ros import pose2array, obstacle2array, marker2array, start_roslaunch_file
from .utils.params import match_in_xml, goal2array
from .utils.info import *
from .utils.tracking import Sort, KalmanTracker
from .utils.format import format_array
from typing import List


class MengeGym(gym.Env):
    """
    Custom gym environment for the Menge_ROS simulation
    """
    metadata = {'render.modes': ['human']}

    def __init__(self, scenario_xml: str):
        """

        :param scenario_xml: menge simulator scenario project file
        """

        super(MengeGym, self).__init__()
        rp.loginfo("Initializing environment")

        assert path.isfile(scenario_xml), 'No valid scenario_xml specified'
        self.scenario_xml = scenario_xml
        self.goals_array = None
        self.goal = None
        self.robot_radius = None
        self._initialize_from_simulator()

        self.timeout = 120
        self.time_step = 0.05  # do 50 ms step
        self._n_observations = 10
        v_max = 10
        num_speeds = 5
        num_angles = 16

        # from config
        self.success_reward = 1
        self.collision_penalty_crowd = -0.25
        self.discomfort_dist = 0.5
        self.discomfort_penalty_factor = - self.collision_penalty_crowd / self.discomfort_dist
        self.collision_penalty_obs = -0.25
        self.clearance_dist = 0.5
        self.clearance_penalty_factor = - self.collision_penalty_obs / self.clearance_dist

        rp.loginfo("Start Menge simulator launch file")
        launch_cli_args = {'project': self.scenario_xml,
                           'timeout': self.timeout,
                           'timestep': self.time_step}
        self._sim_process = start_roslaunch_file('menge_vis', 'menge.launch', launch_cli_args)

        # simulation controls
        rp.logdebug("Set up publishers and subscribers")
        rp.init_node('MengeSimEnv', log_level=rp.DEBUG)
        self._pub_run = rp.Publisher('run', Bool, queue_size=1)
        self._step_done = False

        # observation space
        self._crowd_poses = []  # type: List[np.ndarray]
        self._robot_poses = []  # type: List[np.ndarray]
        self._static_obstacles = np.array([], dtype=float)
        self.rob_tracker = None
        self.ped_tracker = Sort()
        self.combined_state = np.array([], dtype=float)

        # rp.Subscriber("crowd_pose", PoseArray, self._crowd_pose_callback)
        rp.Subscriber("crowd_expansion", MarkerArray, self._crowd_expansion_callback)
        rp.Subscriber("laser_static_end", PoseArray, self._static_obstacle_callback)
        rp.Subscriber("pose", PoseStamped, self._robot_pose_callback)
        rp.Subscriber("done", Bool, self._done_callback)

        # action space
        # from paper RGL for CrowdNav --> 5 speeds (0, v_pref] and 16 headings [0, 2*pi)
        # exponentially distributed speeds
        self._velocities = np.logspace(0, np.log(v_max + 1), num_speeds, endpoint=True, base=np.e) - 1
        # linearly distributed angles
        # make num_angles odd to ensure null action (0 --> not changing steering)
        num_angles = num_angles // 2 * 2 + 1
        self._angles = np.linspace(-np.pi, np.pi, num_angles, endpoint=True)
        self.action_space = spaces.MultiDiscrete([num_speeds, num_angles])

        self._cmd_vel_pub = rp.Publisher('/cmd_vel', Twist, queue_size=1)
        self._advance_sim_srv = rp.ServiceProxy('/advance_simulation', RunSim)

        # initialize time
        # self._run_duration = rp.Duration(self.time_step)
        self._rate = rp.Rate(10)

    def _initialize_from_simulator(self):
        scenario_xml = self.scenario_xml
        scenario_dir = path.split(scenario_xml)[0]

        scene_xml = match_in_xml(scenario_xml, attrib_name='scene')
        if not path.isabs(scene_xml):
            scene_xml = path.join(scenario_dir, scene_xml)
        assert path.isfile(scene_xml), 'Scene file specified in scenario_xml non-existent'

        # extract robot radius from behavior_xml file
        self.robot_radius = float(match_in_xml(scene_xml, tag="Common", attrib_name="r", constraints={"external": "1"}))

        behavior_xml = match_in_xml(scenario_xml, attrib_name='behavior')
        if not path.isabs(behavior_xml):
            behavior_xml = path.join(scenario_dir, behavior_xml)
        assert path.isfile(behavior_xml), 'Behavior file specified in scenario_xml non-existent'

        # extract goal set from behavior file
        goals = match_in_xml(behavior_xml, tag="Goal", return_all=True)
        goals_array = np.array(list(map(goal2array, goals)))
        # sample random goal
        self.goal = goals_array[np.random.randint(len(goals_array))]
        self.goals_array = goals_array

    # def _crowd_pose_callback(self, msg: PoseArray):
    #     # transform PoseArray message to numpy array
    #     rp.logdebug('Crowd Pose subscriber callback called')
    #     pose_array = np.array(list(map(pose2array, msg.poses)))
    #     # update list of crowd poses + pointer to current position
    #     crowd_pose_i = self._crowd_pose_i
    #     self._crowd_poses[crowd_pose_i] = pose_array
    #     self._crowd_pose_i = (crowd_pose_i + 1) % self._n_observations

    def _crowd_expansion_callback(self, msg: MarkerArray):
        # transform MarkerArray message to numpy array
        rp.logdebug('Crowd Expansion subscriber callback called')
        pose_array = np.array(list(map(marker2array, msg.markers)))
        # update list of crowd poses + pointer to current position
        self._crowd_poses.append(pose_array)

    def _static_obstacle_callback(self, msg: PoseArray):
        rp.logdebug('Static Obstacle subscriber callback called')
        # transform PoseArray message to numpy array
        self._static_obstacles = np.array(list(map(obstacle2array, msg.poses)))

    def _robot_pose_callback(self, msg: PoseStamped):
        rp.logdebug('Robot Pose subscriber callback called')
        # extract 2D pose and orientation from message
        robot_pose = msg.pose
        robot_x = robot_pose.position.x
        robot_y = robot_pose.position.y
        robot_omega = 2 * np.arccos(robot_pose.orientation.w)

        # update list of robot poses + pointer to current position
        self._robot_poses.append(np.array([robot_x, robot_y, robot_omega]).reshape(-1, 3))

    def _done_callback(self, msg: Bool):
        rp.logdebug('Done message received')
        self._step_done = msg.data

    def step(self, action: np.ndarray):
        rp.logdebug("Performing step in the environment")

        # # only keep most recent poses before updating simulation
        # self._crowd_poses = list(self._crowd_poses[-1])
        # self._robot_poses = list(self._robot_poses[-1])

        self._take_action(action)

        reward, done, info = self._get_reward_done_info()

        # in first iteration, initialize Kalman Tracker for robot
        if not self.rob_tracker:
            self.rob_tracker = KalmanTracker(self._robot_poses[0][:, :3])

        # update velocities
        for (robot_pose, crowd_pose) in zip(self._robot_poses, self._crowd_poses):
            rp.logdebug("Robot Pose (Shape %r):\n %r" % (robot_pose.shape, robot_pose))
            rp.logdebug("Crowd Pose (Shape %r):\n %r" % (crowd_pose.shape, crowd_pose))
            state = np.concatenate((robot_pose[:, :3], crowd_pose[:, :3]), axis=0)
            ped_trackers = self.ped_tracker.update(crowd_pose[:, :3])
            self.rob_tracker.predict()
            self.rob_tracker.update(robot_pose[:, :3])
        rob_tracker = self.rob_tracker.get_state()
        trackers = np.concatenate((np.concatenate((rob_tracker, [[0]]), axis=1).reshape(1, -1), ped_trackers), axis=0)
        combined_state = trackers[trackers[:, -1].argsort()]
        self.combined_state = combined_state

        ob = (combined_state, self._static_obstacles, self.goal)

        # reset last poses
        self._crowd_poses = []
        self._robot_poses = []

        return ob, reward, done, info

    def _take_action(self, action: np.ndarray):
        """
        execute one time step within the environment
        """
        rp.logdebug("Taking action")

        # in menge_ros the published angle defines an angle increment
        vel_msg = Twist()
        vel_msg.linear.x = self._velocities[action[0]]  # vel_action
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = self._angles[action[1]]  # angle_action

        rp.logdebug("Calling Service")
        # advance simulation by one step
        while not self._advance_sim_srv(1):
            rp.logwarn("Simulation not paused, service failed")
            self._pub_run.publish(Bool(data=False))
        # wait for response from simulation, in the meantime publish cmd_vel
        while not self._step_done or not self._crowd_poses or not self._robot_poses:
            rp.logdebug("Publishing cmd_vel message")
            self._cmd_vel_pub.publish(vel_msg)
            rp.logdebug('Done %r, #Crowd %d, #Rob %d' % (self._step_done, len(self._crowd_poses), len(self._robot_poses)))
            self._rate.sleep()
        rp.logdebug('Done %r, #Crowd %d, #Rob %d' % (self._step_done, len(self._crowd_poses), len(self._robot_poses)))
        self._step_done = False

        # self._pub_run.publish(Bool(data=True))
        # current_time = start_time = rp.Time.now()
        # while current_time <= start_time + self._run_duration:
        #     self._cmd_vel_pub.publish(vel_msg)
        #     current_time = rp.Time.now()
        # self._pub_run.publish(Bool(data=False))

    def _get_reward_done_info(self) -> (float, bool, object):
        """
        compute reward and other information from current state

        :return:
            reward, done, info
        """

        # crowd_pose = [x, y, omega, r]
        recent_crowd_pose = self._crowd_poses[-1]

        # obstacle_position = [x, y]
        obstacle_position = self._static_obstacles

        # robot_pose = [x, y, omega]
        recent_robot_pose = self._robot_poses[-1]

        robot_radius = self.robot_radius
        goal = self.goal

        crowd_distances = np.linalg.norm(recent_crowd_pose[:, :2] - recent_robot_pose[:, :2], axis=1)
        crowd_distances -= recent_crowd_pose[:, -1]
        crowd_distances -= robot_radius

        obstacle_distances = np.linalg.norm(obstacle_position - recent_robot_pose[:, :2], axis=1) - robot_radius

        d_min_crowd = crowd_distances.min()

        d_min_obstacle = obstacle_distances.min()

        d_goal = np.linalg.norm(recent_robot_pose[:, :2] - goal[:2]) - robot_radius - goal[-1]

        # sim node terminated
        if '/menge_sim' not in rosnode.get_node_names():
            reward = 0
            done = True
            info = Timeout()
        # collision with crowd
        elif d_min_crowd < 0:
            reward = self.collision_penalty_crowd
            done = True
            info = Collision('Crowd')
        # collision with obstacle
        elif d_min_obstacle < 0:
            reward = self.collision_penalty_obs
            done = True
            info = Collision('Obstacle')
        # goal reached
        elif d_goal < 0:
            reward = self.success_reward
            done = True
            info = ReachGoal()
        # too close to people
        elif d_min_crowd < self.discomfort_dist:
            # adjust the reward based on FPS
            reward = (d_min_crowd - self.discomfort_dist) * self.discomfort_penalty_factor * self.time_step
            done = False
            info = Discomfort(d_min_crowd)
        # too close to obstacles
        elif d_min_obstacle < self.clearance_dist:
            # adjust the reward based on FPS
            reward = (d_min_obstacle - self.clearance_dist) * self.clearance_penalty_factor * self.time_step
            done = False
            info = Clearance(d_min_obstacle)
        else:
            reward = 0
            done = False
            info = Nothing()
        return reward, done, info

    def reset(self):
        """
        reset the state of the environment to an initial state

        :return: initial observation (ob return from step)
        """
        rp.loginfo("Env reset - Shutting down simulation process")
        self._sim_process.terminate()
        self._sim_process.wait()
        rp.loginfo("Env reset - Starting new simulation process")
        launch_cli_args = {'project': self.scenario_xml,
                           'timeout': self.timeout,
                           'timestep': self.time_step}
        self._sim_process = start_roslaunch_file('menge_vis', 'menge.launch', launch_cli_args)
        self.goal = self.goals_array[np.random.randint(len(self.goals_array))]

        # perform idle action and return observation
        return self.step(np.array([0, np.median(range(self.action_space.nvec[1]))], dtype=np.int32))[0]

    def render(self, mode='human', close=False):
        """
        render environment information to screen
        """
        if close:
            self.close()
        trackers = self.combined_state
        if len(trackers):
            rp.loginfo('Tracked Objects')
            trackers_str = format_array(trackers[:, :-1],
                                        row_labels=trackers[:, -1].astype(int),
                                        col_labels=['x', 'y', 'omega', 'x_dot', 'y_dot', 'omega_dot'])
            rp.loginfo('\n' + trackers_str)
        else:
            rp.logwarn("No objects tracked")
        rp.loginfo('\nNumber of statics obstacles: %d\n' % len(self._static_obstacles))

    def close(self):
        """
        close the environment
        """

        rp.loginfo("Env close - Shutting down simulation process")
        self._sim_process.terminate()
        self._sim_process.wait()
