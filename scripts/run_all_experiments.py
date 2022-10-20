#!/usr/bin/env python3

from __future__ import print_function
from builtins import str
from builtins import range
from builtins import object
from nav_scripts.gazebo_master import MultiMasterCoordinator
from nav_scripts.controller_launcher import ControllerLauncher
from nav_scripts.ros_launcher_helper import RosEnv, GazeboLauncher, RoscoreLauncher
from nav_scripts.gazebo_driver import GazeboDriver
from nav_scripts.movebase_driver import run_test
import nav_quadrotor.scenarios
import nav_quadrotor.quadrotor_impl
import rospy
import time
import random
import datetime
import os
from pathlib import Path
import rospkg
rospack = rospkg.RosPack()

quadrotor = rospack.get_path("nav_quadrotor") + "/launch/spawn_mav_hanging_cylinder.launch"
#TODO: make a proper class encapsulating other robot-specific parameters, etc, such as odometry topic

ControllerLauncher.registerController(name="aerial_pips", filepath=rospack.get_path("nav_quadrotor") + "/launch/aerial_pips.launch")

def rosbag_key_getter(data_dir=None):
    if data_dir is not None:
        rosbag_dir = os.path.join(data_dir, "rosbags")
        rosbag_dir = os.path.expanduser(data_dir)
        Path(rosbag_dir).mkdir(parents=True, exist_ok=True)

        def add_rosbag_key(task):
            task_str = str(datetime.datetime.now()) + '_' + str('%010x' % random.randrange(16 ** 10)) + ".bag"

            rosbag_path = os.path.join(rosbag_dir, task_str)
            rosbag_path = "'" + os.path.expanduser(rosbag_path) + "'"

            if 'controller_args' not in task:
                task['controller_args'] = {}

            task['controller_args']['rosbag_file'] = rosbag_path

        return add_rosbag_key
    else:
        return lambda t : None


def multi_test_runner(tasks, **kwargs):

    start_time = time.time()
    master = MultiMasterCoordinator(**kwargs)
    master.fieldnames.extend(['repeat', 'rosbag_file'])
    master.start()

    master.add_tasks(tasks=tasks)
    master.wait_to_finish()
    master.shutdown()
    end_time = time.time()
    print("Total time: " + str(end_time - start_time))



def get_scenario_tasks(scenarios, set_size, num_sets, show_gazebo, record_rosbag, data_dir, extra_args=None):
    if not isinstance(scenarios, list):
        scenarios = [scenarios]

    add_rosbag_key = rosbag_key_getter(data_dir=data_dir)
    def getTasks():
        controller_freq = 5
        for s in range(num_sets):
            for scenario in scenarios:
                for repeat in range(s*set_size, (s+1)*set_size):
                    task = {'controller': 'aerial_pips', 'repeat': repeat, 'scenario': scenario, 'robot': quadrotor,
                         'record': False, 'timeout': 360, 'world_args': {"gazebo_gui":show_gazebo},
                            'controller_args': {'controller_freq': controller_freq}, 'robot_impl': "quadrotor"}
                    if record_rosbag:
                        add_rosbag_key(task=task)
                    if extra_args is not None:
                        task.update(extra_args)
                    yield task

    return getTasks



if __name__ == "__main__":
    data_dir = "~/simulation_data"      #Root directory for recorded data
    record_rosbag = True                #Should a rosbag be recorded containing details about each run? See 'nav_quadrotor/launch/aerial_pips.launch for recorded topics'
    show_gazebo = True                  #Display Gazebo's GUI while running experiments?
    scenarios = ['hall_obstacle_course', 'demo_gap']    #Which scenarios should be run?
    set_size = 25                        #How many times should each scenario be run before switching to the next?
    num_sets = 4                        #How many times should the entire set of experiments be repeated?

    multi_test_runner(tasks=get_scenario_tasks(scenarios=scenarios, num_sets=num_sets, set_size=set_size, show_gazebo=show_gazebo, record_rosbag=record_rosbag,
                                                        data_dir=data_dir)(), num_masters=1, save_results=True, use_existing_roscore=False, data_dir=data_dir)



