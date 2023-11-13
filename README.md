# Lab_7

## Introduction

The purpose of this lab is to familiarize you with the UR3 robot and the tools we have for making it do useful and/or interesting things. Additionally, you will cause the end point of the robot arm to move in both a square and circle in a vertical plane and in a horizontal plane. The Horizontal Plane is the plane perpendicular to Z-Axis and the Vertical plane is the plane perpendicular to either X or Y axes. Next, you will cause the end point of the arm to move in the largest possible square that is not in the horizontal plane.

**NOTE**: Throughout the course you will **first** create a successful simulation of the desired arm movement in Gazebo. Only after getting this simulation approved by the lab staff will you implement it on the actual arm. This is a very important safety measure.

Throughout this part of the course, you will use a collection of tools that we have provided. These include Docker containers, ROS, Moveit, and Gazebo. The basics of these tools are explained in the lectures. These tools are already loaded on the lab computers.

All the tools you need to do this lab are in a Docker container. Docker containers are built on images which are built from dockerfiles. Your system in the lab has the docker image with all the required tools such as ROS and Moveit! Installed. You just need to build a container using that image and work inside the container. To save the work, you will volume map a directory from the host pc to the docker container. Any changes you make in the mapped directory inside the container will reflect in the directory in the host pc. To learn more about docker visit [docker documentation](https://docs.docker.com/).

**Volume Map**: Docker Containers are destroyed when you exit the container, which means all the data will be lost. Volume mapping is used to save the important data before destroying the container. A directory from host pc is mapped to a directory in the container. Changes made in the mapped directory in the docker container is reflected in the host pc. Save the important data in the mapped directory inside the container.

## Docker and docker image

If you want to work on your own computer, install docker and portainer (optional) using [this page](https://github.com/ENRE467/Getting_Started/wiki/Installing-Docker-and-Portainer) and build a docker image using [this page](https://github.com/ENRE467/Getting_Started/wiki/Building-a-Docker-Image)

## Steps

**The commands given in each step below are meant to be copied and pasted in the terminal**.

1. Remember to create your own folder on the lab machine so you can save your work. Also, just to be sure, upload your work to your Github account before you leave the lab. Do this in the host pc and not inside the Docker container as `git push` and `git pull` commands will not work inside the container. This is because your git repositories does not exist inside the docker container.

1. Open a terminal window by pressing `Ctrl + Alt + T`. In the terminal window, navigate to your folder using cd command. Now, run the following command to clone the repository for Lab 7:

    ```bash
    git clone https://github.com/ENRE467/Lab_7.git
    ```

1. Navigate to the `Lab_7/src` directory in your folder using cd command.

1. Visual Studio Code (VSC) is already installed in your systems in the lab. You will use VSC to write code in this part of the course. To open the src directory in VSC, run the following command:

    ```bash
    code .
    ```

1. Run the following command so that you can see the GUI applications from docker container in the screen of the host pc:

    ```bash
    xhost +local:docker
    ```

1. Now, you will create a docker container based on the `ur3e_image` image which is already on your lab computer and volume map the `src` directory in the host pc to the `src` directory in the docker container. To do that, enter the following command (Make sure that you are in the `Lab_7/src` directory inside the terminal before running this command):

    ```bash
    docker run -it --rm --name UR3Container --net=host --ipc=host --pid=host --privileged --env="DISPLAY=$DISPLAY" --volume="$PWD:/home/${USER}/catkin_ws/src" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="/dev:/dev:rw" --ulimit rtprio=99 --ulimit rttime=-1 ur3e_image:latest
    ```

1. Install the required dependencies

    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```

1. Now, you are in the workspace directory in the docker container. This is your catkin workspace. Check that the `~/catkin_ws/src` directory contains the files from the `Lab_7/src` directory in your host pc by using the command `ls ~/catkin_ws/src`. This will list all the files in your src folder. Now, if everything seems good, the first thing you do is build your catkin packages. To do that, first go to the workspace directory (if you are not already there) using the command `cd ~/catkin_ws`. To build the workspace, execute the following command:

    ```bash
    catkin build
    ```

    After the packages are built, you need to source them so that you can use them in the current terminal window. Run the following command to do that:

    ```bash
    source ~/catkin_ws/devel/setup.bash
    ```

    To build projects in ROS, it is advised to follow the specific directory structure. place all you code files in the src folder of the catkin project. see below for reference.

    ```bash
    ┌──────────────────────────┐             ┌──────────────────────────┐
    │ ProjectDir               │             │ catkin_ws                │
    │ │                        │             │   │                      │
    │ └────src                 │             │   └──src                 │
    │      │                   │             │      │                   │
    │      ├─catkin_project_1  │             │      ├─catkin_project_1  │
    │      │                   │             │      │                   │
    │      ├─catkin_project_2  ├────────────►│      ├─catkin_project_2  │
    │      │        x          │             │      │        x          │
    │      │        x          │ Volume Map  │      │        x          │
    │      │        x          │             │      │        x          │
    │      │        x          │             │      │        x          │
    │      └─catkin_project_N  │             │      └─catkin_project_N  │
    │                          │             │                          │
    └──────────────────────────┘             └──────────────────────────┘
    ```

    Note: ROS drivers and description packages are located in another folder (`~/ros_ur_driver`) in the docker image. They are already compiled and sourced in the `.bashrc` file. These packages were separated to minimize the build time for your code.

1. Tmux is a tool which is used to split a terminal window into multiple terminals. Tmux is already installed in your docker container. To split the terminal vertically, type tmux and press enter, this will open the current terminal with tmux, then click on the terminal you want to split and press `Ctrl + A` to select that terminal and press `V` to split it vertically. To split the terminal horizontally, click on the terminal you want to split and press `Ctrl + A` to select it and then press `B` to split it horizontally to do it manually. 
An example command to split into four terminals using terminal commands is below:

    ```bash
    tmux new-session \; \split-window -v \; \split-window -h \; \select-pane -t 1 \; \split-window -h
    ```

1. Run the following command to start Gazebo with the UR3e arm in it:

    ```bash
    roslaunch ur3e_setup ur3e_gazebo.launch z_height:=0.77
    ```

    `z_height` is the height at which the robot is spawned in Gazebo.

1. In a different terminal window, run the following command to start MoveIt functionality and RViz:

    ```bash
    roslaunch ur3e_moveit_mrc ur3e_moveit.launch sim:=true
    ```
    
    This also spawns a back wall obstacle that is needed for safety reasons. However, if the wall has to be removed, it can be done by adding `spawn_wall:=false` to the end of the command.

1. The `moveit_tutorial` package has sample code for performing three tasks: 1. Move the robot to a joint goal, 2. Move the robot to a pose goal and 3. Move the robot from one point to another in a cartesian path. You can refer to the `tutorial.cpp` in the `moveit_tutorial` package for the sample code. This sample code uses the helper functions from `moviet_wrapper` package. In a new terminal, run the following command to run this sample code:

    ``` bash
    rosrun moveit_tutorial tutorial
    ```

1. You will use these helper functions in your code to move your robot in square and circle trajectories. A package for this lab is provided to you and the name of this package is `ur3e_trajectory`. Add your code to the files `square.cpp` and `circle.cpp` for square and circle trajectories. 

    Run the following command to run your code for square or circle trajectories:

    ``` bash
    rosrun ur3e_trajectory square 
    ```

    >Note: Replace square with circle if you want to run your circle code.

1. You need to calculate the error between the trajectory followed by your robot and the desired trajectory. To do this, you have to record the end effector positions while your robot traces the trajectory. The `RecordPose.cpp` file contains the code to record end effector positions at the rate of 2 Hz. It starts recording poses when the boolean parameter `record_pose` turns true. You have to set the value of this parameter to true before executing the trajectory and set it to false after trajectory executions. Look at the end of `tutorial.cpp` file in the `moveit_tutorial` package for sample implementation. The boolean parameter `record_pose` needs to be loaded to parameter server and the `RecordPose.cpp` program will look for that parameter from the parameter server. Run the following command to load the parameter:

    ```bash
    roslaunch ur3e_trajectory load_params.launch
    ```

    Edit the string variable `out_path` in the `RecordPose.cpp` file to the destination where you want to save your end effector poses. After this is done, Run the following command at the same time you run your code for square or circle trajectory:

    ```bash
    rosrun ur3e_trajectory RecordPose
    ```

    You can use the generated csv file of the end effector poses to plot the followed trajectory against the ideal trajectory.

1. After you are done with your simulation. You can run your code on the real UR3e arm. Ask one of the Teaching Assistants to help you.

## Commands to run your code on real UR3e robot

The steps for running your code on the real UR3e arm are similar in their structure to the ones used to for the Gazebo simulation.

1. Start docker container from the `Lab_7/src` folder:

```bash
docker run -it --rm --name UR3Container --net=host --ipc=host --pid=host --privileged --env="DISPLAY=$DISPLAY" --volume="$PWD:/home/${USER}/catkin_ws/src" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="/dev:/dev:rw" --ulimit rtprio=99 --ulimit rttime=-1 ur3e_image:latest
```
2. Build the code, source it, and start tmux:

```bash
catkin build
source devel/setup.bash
tmux new-session \; \split-window -v \; \split-window -h \; \select-pane -t 1 \; \split-window -h
```
3. Copy the calibrated for each arm kinematics configuration file from the `Desktop/ENEE467 IMPORTANT` folder to the `Lab_7/src/ur3e_setup/config` folder.

4. Instead of Gazebo launch ROS drivers to connect to the robot arm:

```bash
roslaunch ur3e_setup ur3e_bringup_mrc.launch robot_ip:=192.168.77.22 kinematics_config:=$(rospack find ur3e_setup)/config/ur3e_calib.yaml z_height:=0.77
```
**Attention**: If there are any warnings or errors in the output, stop immidately and contact your TA or the lab manager.

5. In one of the tmux termninals run 

```bash
rostopic list
```
to make sure that the connection was established successfuly and you have access to ROS topics.

5. Ask your TA to help launch the `ur3e_ros` program on the UR3e tablet. Note, at this point you are controlling the arm from your computer. The terminal with ROS drivers should print the following text `Robot connected to reverse interface. Ready to receive control commands.`

6. Start Moveit! with Rviz for UR3e:

```bash
roslaunch ur3e_moveit_mrc ur3e_moveit.launch
```
