# teleop_twist_keyboard_cpp

## Overview

🚗🚗🚗 This ROS package provides teleoperation functionality for a mobile robot using keyboard inputs. It allows users to control the robot's movement by mapping specific keys to velocity commands. The package is designed to be simple and easy to use, making it suitable for quick testing and prototyping. 🚗🚗🚗

**Keywords:** hero_chassis_controller, ROS, ros_control, mobile robot, pluginlib

### License

📜📜📜 The source code is released under a [BSD 3-Clause license](LICENSE.txt). 📜📜📜

**Author:** Mao Luo  
**Affiliation:** [Mao Luo](https://github.com/gdut-dynamic-x/simple_chassis_controller/blob/master)  
**Maintainer:** Mao Luo, 3214283533@qq.com

🛠️🛠️🛠️ The Hero Chassis Controller package has been tested on ROS Indigo and Noetic on Ubuntu 20.04. This is research code, and changes may occur frequently; fitness for any particular purpose is disclaimed. 🛠️🛠️🛠️


## Installation

### Building from Source

#### Dependencies

📚📚📚 - [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)
- [Eigen](http://eigen.tuxfamily.org) (linear algebra library) 📚📚📚

Install dependencies:

```bash
sudo rosdep install --from-paths src
```

#### Building

🔧🔧🔧 To build from source, clone the latest version from this repository into your catkin workspace and compile the package: 🔧🔧🔧

```bash
cd ~/catkin_ws/src
git clone https://github.com/maoluois/teleop_twist_keyboard.git
cd ../
rosdep install --from-paths . --ignore-src
catkin_make
```

### Running in Docker

🐋🐋🐋 Docker is a convenient way to run an application with all dependencies bundled together. First, [install Docker](https://docs.docker.com/get-docker/). 🐋🐋🐋

Spin up a simple container:

```bash
docker run -ti --rm --name ros-container ros:noetic bash
```

Now, create a catkin workspace, clone the package, build it, and source the setup:

```bash
apt-get update && apt-get install -y git
mkdir -p /ws/src && cd /ws/src
git clone https://github.com/maoluois/teleop_twist_keyboard.git
cd ..
rosdep install --from-path src
catkin_make
source devel/setup.bash
rosrun teleop_twist_keyboard_cpp teleop_twist_keyboard_cpp_node
```

## Usage

🎛️🎛️🎛️ Run the main node: 🎛️🎛️🎛️

```bash
rosrun teleop_twist_keyboard_cpp teleop_twist_keyboard_cpp_node
```

```txt
Reading from the keyboard  and Publishing to Twist!
---------------------------
translation slipping:
        w
   a    s    d

---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-x to quit
```

#### Published Topics

- **`/cmd_vel`** ([geometry_msgs/Twist]): Velocity commands for the robot.

#### Parameters

- **`speed`** (double): initial speed. Default: `0.5`.
- **`turn`** (double):  Initial turn rate. Default: `1.0`.
- **`speed_limit`** (double): Maximum speed limit. Default: `1000`.
- **`turn_limit`** (double): Maximum turn rate limit. Default: `1000`.
- **`repeat_rate`** (double): Rate at which commands are repeated. Default: `1000.0`.
- **`key_timeout`** (double): Timeout for key inputs. Default: `0.5`.

## Bugs & Feature Requests

🐞🐞🐞 Please report bugs and request features using the [Issue Tracker](https://github.com/maoluois/teleop_twist_keyboard.git). 🐞🐞🐞

[ROS]: http://www.ros.org
[Eigen]: http://eigen.tuxfamily.org
[geometry_msgs/Twist]: http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
[nav_msgs/Odometry]: http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html

