To Run the Pangolin based simulation, run the following commands:

1) `cd Code`
2) ` python3 vio.py --view --path ./MH_01_easy`

This will launch a window that will simulate the entire simulation.

For Ground Truth and RTE Error Calculations, we have used `rpg_trajectory_evaluation` repository.
To make the `groundtruth.txt` run the `asl_groundtruth_to_pose.py` with the `groundtruth csv`
We generate the `estimated.txt` with the simulation itself.

Putting these two `.txt` in the same folder and then running `python3 analyze_trajectory_single.py <result_folder> ` will lead to the resulting error plots.


