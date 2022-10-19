#!/usr/bin/env python3

from __future__ import print_function
from builtins import str
from builtins import range
from builtins import object
from nav_scripts.gazebo_master import MultiMasterCoordinator
from nav_scripts.testing_scenarios import TestingScenario, TestingScenarios, GeneralScenario
from nav_scripts.controller_launcher import ControllerLauncher
from nav_scripts.ros_launcher_helper import RosEnv, GazeboLauncher, RoscoreLauncher
from nav_scripts.gazebo_driver import GazeboDriver
from nav_scripts.movebase_driver import run_test
import nav_quadrotor.scenarios
import nav_quadrotor.quadrotor_impl
import rospy
from geometry_msgs.msg import PoseStamped
import time
import csv
import math
import rospkg
rospack = rospkg.RosPack()


def run_single_task(scenario):
    rospy.init_node('test_driver', anonymous=True)
    rospy.Rate(1).sleep()

    task = {'scenario': scenario, "robot_impl": "quadrotor"}
    #task["action_server_wait_time"]=0

    scenarios = TestingScenarios()
    scenario = scenarios.getScenario(task)

    rospy.Rate(1).sleep()

    start_time = time.time()
    scenario.setupScenario()
    end_time = time.time()

    print(str(end_time - start_time))

    try:
        result = run_test(task=task, goal_pose=scenario.getGoalMsg())
        print(result)
        if isinstance(result, dict):
            task.update(result)
        else:
            task["result"] = result
        return task
    except rospy.exceptions.ROSInterruptException as e:
        pass
    return None



if __name__ == "__main__":
    ##Comment out one or the other scenario:
    scenario = "demo_gap"
    #scenario = "hall_obstacle_course"

    run_single_task(scenario=scenario)