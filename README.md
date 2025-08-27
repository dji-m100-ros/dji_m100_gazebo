dji_m100_gazebo
=========

A ROS package for simulating the DJI Matrice 100 Drone in Gazebo. Initial work is made in [here](https://github.com/caochao39/hku_m100_gazebo). On top of that I have added a Gazebo plugin compatible with ROS Melodic. 

**Installation**

1. Clone this repo into your catkin workspace
2. Ensure you are in the root of your workspace (e.g. ~/ws/).
3. Pull source dependencies first:
```
wstool init src
wstool merge -t src src/dji_m100_gazebo/dep.rosinstall
wstool update -t src
```
4. Install system dependencies:
```
sudo rosdep init || true
rosdep update --include-eol-distros
rosdep install --from-paths src --ignore-src -r -y --rosdistro <YOUR_DISTRO>
```
5. Finally build with `catkin build`

**Configuration**
  * Turn on DJI Matrice 100 and connect it to the PC simulator


**Usage**

Execute the one-liner.
> $ roslaunch dji_m100_gazebo simulate.launch