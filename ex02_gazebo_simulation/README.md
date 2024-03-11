# ex02_gazebo_simulation


## Download

Git-Clone the following packages into your ROS-workspace `src` directory using git:
You will need some package to run the simulation of the robot. The same software packages are installed on the robot as well.

- https://github.com/uos/ceres_robot
- https://github.com/uos/uos_tools
- https://github.com/uos/epos2_motor_controller
- https://github.com/uos/volksbot_driver

The current main branch of those software packages should be compatible with ROS2 humble. To make sure, switch all the repositories to "humble" branch instead: Go to a cloned repository folder and run `git checkout humble`. After you did that for every repository, compile your workspace. If errors occur:
- ROS-Package is missing: `sudo apt-get install ros-humble-MISSING-PACKAGE`
- System package is missing: `sudo apt-get install libftdipp1-dev`

### What to do when packages are missing?

Let's assume the package `missing_package` is missing from your computer. Try to install it in this order, finish when one command has succeeded:

1. Install via apt: `sudo apt install ros-humble-missing-package`
2. Install via Github: Search for the package on github. Check if it's a valid ROS2-humble package. Download `missing_package` into your `src`-folder and compile it
3. Install via Gitlab: Search for the package on UOS-Gitlab. Check if it's a valid ROS2-humble package. Download `missing_package` into your `src`-folder and compile it
4. Go to Alex

## Run the Simulation

```console
ros2 launch ceres_gazebo ceres_gazebo_launch.py
```

A graphical interface starts with a robot in it, spawned in an office environment. Try to add a cube. Try to move the robot. Make yourself familiar with the functionalities. The simulation, as the name tells, simulates a real robot. So when you learn to control the robot in the simulation, you will be able to control the robot in the real world. The simulation will generate the sensor data and publish them on some topics. Find out which topics. Print out some of the messages that are streamed on those topics.

## Control the robot

```console
ros2 launch uos_diffdrive_teleop key.launch
```

A window will open. As long as it is the activate window you can control the robot via the shown keys. Move the robot through the office world.

## Visualize the Sensor Data

```console
ros2 run rviz2 rviz2
```

or the shortcut:

```console
rviz2
```

The robot is equipped with a laser scanner. Try to visualize the corresponding topic. Set the fixed frame to 'base_link'. What a fixed frame is will be explained soon.
Move the robot around and see how the sensor data changes. RViz will be used with the real robot to monitor processes. Gazebo not.

## Use the sensor data

Write a node that filters noise from the scan. Subscribe to the topic `scan`, average the distances with the direct neighboring measurements (average of 3 measurements in total) and publish the results on a new topic `scan_filtered`. Visualize the results with RViz. 

### Point Clouds

Messages on the topic are in polar coordinates. Write a node that converts the LaserScan to a `PointCloud2` message and publishes it on the topic `scan_cloud`. Don't use external libraries for the conversions. Copy the header of the `LaserScan` message to the header of the `PointCloud2` message. After publishing, visualize the results with RViz.


## TF

Make yourself familiar with the concept of transformations. Visualize the transformations via RViz. In ROS2 there is a library called tf2 that handles all the transformations. Internally they present all the transformations as a tree. You can visualize it by calling

```console
ros2 run rqt_tf_tree rqt_tf_tree
```

If it's not installed you can install it via `sudo apt install ros-humble-rqt-tf-tree`.
Read the docs: https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html

Every piece of sensor data was recorded in a certain reference system at a certain time. Therefore, sensor data messages usually have a field `std_msgs/Header`:

```cpp
...
string frame_id
time stamp
```

Sometimes messages have the postfix `Stamped` if they are extending a message only by a header. For example, there is a message `geometry_msgs/Point`:

```cpp
float64 x
float64 y
float64 z
```

And the corresponding `geometry_msgs/PointStamped`:

```cpp
std_msgs/Header header
geometry_msgs/Point point
```

Only if the stamped message is published, RViz knows where to visualize the point correctly. It transforms this point into the `fixed frame` that is configured in the GUI.

### Use TF

Node: `transform_pcl`.
Short Description: Transform a PointCloud2. Read a cloud from the recently generated point cloud topic and transform it into another coordinate frame. What you will need:

- tf2 transform listener: to receive the transformation between two frames. Google for examples
- apply the transformation to each of the point cloud's points
- create a new point cloud from the transformed points
- change the header of the new point cloud to the new coordinate frame
- publish the resulting cloud on the topic `scan_cloud_transformed`.

Show the results in RViz. Explain the results.

## ROS2 params and remapping

ROS has also a mechanism to change parameters during runtime. For example, we want to enable the user to decide at runtime to which frame the cloud should be transformed. Make yourself familiar with ROS2 parameters. Add a parameter called `target_frame` to your `transform_pcl` node. Call your node from the command line and change the parameter:


```console
ros2 run ex02_gazebo_simulation transform_pcl --ros-args -p target_frame:=imu
```

How to exactly use parameters in your source code is explained here: https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP.html

Now the node should transform the point cloud to a different coordinate system. Check it by showing the results in RViz and printing the topic's header:

```console
ros2 topic echo /scan --field header
```

The next mechanism is called "remapping". Imagine one user wants to use different topics than those you wrote into your source code. Remapping can change those topics at runtime. For example, if you want your `scan_to_pcl` node to publish its results on `scan_cloud2` instead of `scan_cloud`:

```console
ros2 run ex02_gazebo_simulation scan_to_pcl --ros-args -r scan_cloud:=scan_cloud2
```

The same can be done for the input scan:

```console
ros2 run ex02_gazebo_simulation scan_to_pcl --ros-args -r scan:=scan2
```

The node will subscribe on the topic `scan2` instead of `scan`. Like this, you can convert scans to clouds from different topics without changing the code.

See the docs more in-depths explanations: https://docs.ros.org/en/foxy/How-To-Guides/Node-arguments.html

## Launch Files

You might have been confused about the `ros2 launch ...`-commands. With `ros2 run` you can start simple nodes. With `launch` you can start so-called launch-files instead.
Launch files are placed within a package's launch folder. Create a launch file called `launch_my_nodes.xml`:

```bash
ex02_gazebo_simulation/
  launch/
    launch_my_nodes.xml
```

With launch-files you can capsule node runs into one file. One advantage of this is that you will save a lot of terminal tabs.

Task: Try to put all your nodes into the launch file and start it. A tutorial how to do it: https://docs.ros.org/en/foxy/How-To-Guides/Launch-file-different-formats.html.
Change your parameters and test how to do remapping inside of a launch file.


After creating the launch file, install the launch folder by adding the following lines to your `CMakeLists.txt`:

```cmake
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
```

Then compile your workspace and run your launch file with

```console
ros2 launch ex02_gazebo_simulation my_launch_file.xml
```

## Drive the robot (Fun part)

Write a node that at first just publishes a message of type `geometry_msgs/Twist` on the topic `cmd_vel`. Figure out what fields of the message are responsible for letting the robot move.

### Addition: Free Space

Use the laser scan to find free space. Steer the robot towards it and drive. What is needed:

- Subscriber to `scan`
- Publisher on `cmd_vel`
