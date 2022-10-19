#!/usr/bin/env python3
from nav_quadrotor.result_analyzer import CollisionData, main as analyze

if __name__=="__main__":
    data_dir = "~/simulation_data"  #Directory containing results files (should be the same as 'data_dir' in run_all_experiments.py)
    results_file = None             #Path of the results file to analyze; if None, selects the most recent results

    analyze(data_dir=data_dir, results_file=None)