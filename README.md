# ros2_tutorial

Tutorials for ROS2 - humble.

- Ubuntu 22 (Only native!)
- ROS2 humble
- C++

## [ex01_first_package](./ex01_first_package/README.md)

- Learning ROS concepts
- Writing a simple subscriber and publisher
- Learning C++ and CMake by implementing subscribers and publishers in different ways
- Learn to use `vim`
- Environment variables, `.bashrc`

## [ex02_gazebo_simulation](./ex02_gazebo_simulation/README.md)

- Start a robot! Virtually
- More ROS packaging
- RViz2
- New message: LaserScan
- Transformations: Concept, Implementation
- New message: PointCloud2
- ROS2 parameters + remapping
- XML launch files
- Fun: Steer the robot
- New message: Twist

## [ex03_state_estimation](./ex03_state_estimation/README.md)

- Concept: State Estimation via
  - Wheel Odometry
  - IMU
  - LiDAR
  - Camera
- Fusion
  - Kalman-Filter
  - `robot_localization`-package
- Start a robot! In reality.
- Tool: `tmux`
- Mapping
  - Maps
  - SLAM

## [ex04_navigation](./ex04_navigation/README.md)

- Localization in given maps
  - EKF vs SLAM vs AMCL
- Nav2
- Actions
- Call Nav2 action servers via C++

## [ex05_behavior_trees](./ex05_behavior_trees/README.md)

!! Unfinished !!

- Overview: Deliberation Layer
- BehaviorTree.CPP
- Simple Behavior Tree
- Simple Behavior Tree with Groot monitor
- Writing Behavior Tree Plugins

TODOs:

- Editing using Groot
- Using the Blackboard

More TODOs in ex05 README.


## Information for Teachers

When the students reached certain points, I explained the theory and some practical things in more detail on the blackboard. This included, for example:

- Introductions
- Sensors
- Kalman-Filter (beginning with 1D example)
- SLAM pose graph incl. different uncertainties for linear and rotational motion
- MCL, kidnapped robot problem, multimodal distributions
- Nav2, layers, search algorithms, ...

Furthermore, I am aware of more nice ROS tools that help to finish things faster. However, I did not mention them because the students had to learn enough commands anyways.

These tutorials were tested by 12 undergraduate students of Osnabrück Univertity. They were in their 3rd or 5th semester and did not have much experience with C++, nor with CMake, or Linux. It took them 6 days to finish with `ex04_navigation`. They then had to work on a project for about 1.5 weeks and give a 30-minute presentation including a demonstration at the end. The remaining days were spent on documentation. In total, it took 3 weeks full-time.
