# Readme
WIP instructions for running the experiments in our ICRA 2023 submission: _AeriaLPiPS: A Local Planner for Aerial Vehicles with Geometric Collision
Checking_. 

## Setting up
Follow [these instructions](https://github.com/ivalab/NavBench/master/ICRA2023/README.md) to setup the necessary workspace and dependencies.

Be sure to source your new workspace (and any `venv` if necessary)



## Reproducing Experimental Results
### Run Experiments
We will be making use of the NavBench framework to automatically run all experiments. If the environment has been configured correctly, then the system will take care of everything: starting a ROS core, launching the worlds, initializing the robot, recording outcomes, etc.
1. Configure parameters near the very end of `<catkin_ws>/src/aerial_pips/scripts/run_all_experiments.py`:
  - `data_dir`: The root directory for recorded results
  - `record_rosbag`: If `True`, certain information about each experiment will be recorded. The list of recorded topics can be found in the file `launch/aerial_pips.launch` within the `nav_quadrotor` package. Depending on your needs, you may wish to modify the list
    - Note: if you want to generate the timing statistics, make sure to keep the topics `rosout_agg`, `/hummingbird/obstacle_avoidance_controller_node/potentials_planner/plan`, and `/hummingbird/bumper`
    - Most other topics should be fairly self-explanatory
    - Including the commented-out topics from the bottom of the file allows the full state of the perception pipeline to be reproduced while replaying experiments at the cost of much larger recorded files.
  - `show_gazebo`: Set to `False` to reduce the overall cpu load if you don't need to watch the experiments run
  - `scenarios`: Specify which scenarios to run tests in; by default the system will run tests in both environments featured in the submission
  - `set_size`: How many times should each scenario be run before switching to the next?
  - `num_sets`: How many times should the entire set of experiments be repeated? (This can be useful to ensure you have some results for each scenario even if you stop the experiments early.)
2. Run the script: `rosrun aerial_pips run_all_experiments.py`
  - The experiments run using a separate ROS core, so in principle they can run concurrently with whatever else you may be doing. However, in general it is best to minimize unnecessary loads on the computer while running the experiments, especially if you want to get accurate timing results.
  - If your computer is sufficiently powerful, you can speed up the benchmarking process by running multiple experiments in parallel. Just change the `num_masters` keyword arg at the bottom of the file. Monitor your load average to ensure that your computer isn't falling behind.
  - If you need/want to stop the experiments early, press `Ctrl+C`. This will instruct the benchmarking system to shutdown all of its processes once any active experiments have finished. If you don't wish to wait for the active experiments to finish, press `Ctrl+C` again, and it will attempt to shutdown immediately.
  - The benchmarking system is fairly robust and should be able to recover from most sources of errors. If you encounter situations where it hangs indefinitely, please create an issue.
  - Ideally, on shutdown the benchmarker cleans up all the processes that it started. If for whatever reason it does not (for example, `gzserver` is still running in the background), then you will need to manually kill the processes.
  
### Analyze Results

