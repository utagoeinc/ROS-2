# Sphero Driving with ROS 2

# Setup
## Install ROS 2 Dashing
Follow the official installation instruction:
https://index.ros.org/doc/ros2/Installation/Dashing/

Make sure source the setup file `/opt/ros/dashing/setup.bash`.
You might want to add the script to `~/.bashrc`.

#### ~/.bashrc
```
source /opt/ros/dashing/setup.bash
```

## Create a ROS 2 Workspace
[ROS.org](https://index.ros.org/doc/ros2/Tutorials/Workspace/Creating-A-Workspace/#ros2workspace)

1. Create a new directory. `$ mkdir -p ros2_ws/src`
2. Build the workspace. `$ colcon build`
3. Source the overlay. `$ source ~/ros2_ws/install/local_setup.bash`
    - You should add this to `~/.bashrc`

## Install PySphero
### Dependencies
```
$ sudo apt-get install libgtk2.0-dev
$ sudo pip3 install bluepy
$ sudo pip3 install gatt
```
### Install PySphero
```
$ sudo pip3 install pysphero
```
Restart the sesion.

## Clone this Repository
Move to `~/ros2_ws/src` directory and then clone this repository.
```
$ cd ~/ros2_ws/src
$ git clone https://github.com/utagoeinc/sphero_driving.git
```

## Build the Package
1. Move to workspace root directory: `$ cd ~/ros2_ws`
2. Build the package.
    - `$ colcon build`
    - Or you can select specific package to build:
      - `$ colcon build --packages-select sphero_driving`

# How to Use
## Preparation
### Find Your Sphero's MAC Address
Find your Sphero's MAC address using `find_toy.py` in the `sphero_driving/sphero_driving/examples` directory.

You need to run this script with sudo permission.
```
$ sudo python3 find_toy.py
```
### Change the MAC Address in drive.py
Change the MAC address in `drive.py` to yours found in the previous step.

## Drive Your Sphero
### Run Subscriber Node
```
$ ros2 run sphero_drive drive
```

### Open RQt Robot Steering Tool
`$ rqt`

Then **rqt** window will pop up.
Select `Plugins > Robot Tools > Robot Steering`.
Set the Topic `/cmd_vel`.
<img src=https://i.imgur.com/1xBjmUq.png width=500/>

# Issue
You can't exit spin loop properly.
This issue is under investigating.
