# A-Star-Path-Planning-Simulated-On-Coppeliasim-ROS2

## Overview

This project focuses on A* Path Planning, a popular pathfinding algorithm used in robotics and artificial intelligence. A* is known for its efficiency and optimality in finding the shortest path from a start point to a goal point on a grid or graph.

The project includes an implementation of the A* algorithm and visualization of its performance on various maps, making it a valuable resource for understanding path planning in a simulated environment.

## A* Algorithm

The A* algorithm is a search algorithm used to find the optimal path from a start node to a goal node in a weighted graph. It combines two essential components:

- **G-Cost (Actual Cost):** The cost to reach the current node from the start node.
- **H-Cost (Heuristic Cost):** An estimation of the cost from the current node to the goal node.

A* selects nodes to expand based on their total cost, which is the sum of G-Cost and H-Cost. By exploring nodes with lower total cost first, A* efficiently finds the shortest path while considering a heuristic to guide its search.

## Flowchart

![Algorithm Flowchart](https://user-images.githubusercontent.com/106331831/236201740-8626b4d5-e1f8-4ea8-aebe-7ddca2a3137e.png)


## Prerequisites:

- Download [Coppeliasim edu v.4.5.1](https://www.coppeliarobotics.com/files/V4_5_1_rev4/CoppeliaSim_Edu_V4_5_1_rev4_Ubuntu22_04.tar.xz).


## Building the environment:
```bash
mkdir ros2_ws && cd ros2_ws
git clone https://github.com/MostafaELFEEL/A-Star-Path-Planning-Simulated-On-Coppeliasim-ROS2.git
colcon build
```

## Running the Simulation:

1. Open CoppeliaSim:
```bash
cd ~/<Path_to_Coppeliasim_Folder> && ./coppeliaSim.sh
```
Make sure to change **<Path_to_Coppeliasim_Folder>** with Coppeliasim folder path.

2. Execute the Python script for 2D or 3D path planning:
```bash
cd && ros2 run path_planning 2dpath_planning
```
or
```
cd && ros2 run path_planning 3dpath_planning
```

## Results of 2D path planning: map number 1, 2, and 3. (only 3 maps)

![image](https://github.com/MostafaELFEEL/A-Star-Path-Planning-Simulated-On-Coppeliasim-ROS2/assets/106331831/4ad2011e-c6b5-46e1-8b69-a54fc7ef0598)


![image](https://github.com/MostafaELFEEL/A-Star-Path-Planning-Simulated-On-Coppeliasim-ROS2/assets/106331831/2cfe8d47-03cb-4798-81f3-1c1def461475)


![image](https://github.com/MostafaELFEEL/A-Star-Path-Planning-Simulated-On-Coppeliasim-ROS2/assets/106331831/0f739be7-4cce-4fbe-b807-a51ee76470e1)

## Results of 3D path planning: map number 1, 2, and 4. (There are 7 maps)

![image](https://github.com/MostafaELFEEL/A-Star-Path-Planning-Simulated-On-Coppeliasim-ROS2/assets/106331831/c510a2df-8766-43c7-825f-942b0919e614)

![image](https://github.com/MostafaELFEEL/A-Star-Path-Planning-Simulated-On-Coppeliasim-ROS2/assets/106331831/ef573883-d48d-4060-87a6-8582c464e189)

![image](https://github.com/MostafaELFEEL/A-Star-Path-Planning-Simulated-On-Coppeliasim-ROS2/assets/106331831/3a9fc7f5-77f2-4ae3-a3c4-fc69b3258bbf)





