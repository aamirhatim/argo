## Luggo - An Autonomous Suitcase that Follows You
#### Northwestern MSR Winter Project 2018
#### Aamir Husain

## Part 1: Requirements
### Hardware
- Raspberry Pi 3 B
- 2 DC Brushed motors
- RoboClaw Motor controller
- 12V battery (2800mAh)
- Raspberry Pi Camera Module v2
- 2 wheels
- Extra features: power switch, caster wheel, camera mount, screws, velcro, adhesives, USB cables (A, micro, mini)

### Software
- ROS Kinetic
- Ubuntu MATE (Rasbian OS also compatible, but it's frustrating to use for ROS)

### ROS packages:
- [raspicam_node](https://github.com/dganbold/raspicam_node) Provides an easy way to interfave the Raspberry Pi camera module with ROS
- [ar_track_alvar](http://wiki.ros.org/ar_track_alvar) (use apt-get) Library for creating, detecting, and tracking AR tags
- [image_view](http://wiki.ros.org/image_view) (use apt-get) Simple viewer for ROS image topics

## Part 2: Installing ROS Kinetic on the Pi (more info [here](http://wiki.ros.org/kinetic/Installation/Ubuntu))
1. Install the Ubuntu MATE [operating system](https://ubuntu-mate.org/raspberry-pi/) onto an SD card and plug it into the Pi. Grab a snack, the initial setup will take a little while.
2. Update and upgrade your Pi:
```
$ sudo apt-get update
$ sudo apt-get dist-upgrade
```
Set your localization settings and restart the Pi.
3. Set up your ROS repositories:
```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```
4. Run `sudo apt-get update`.
5. Install ROS Kinetic (full desktop version):
```
$ sudo apt-get install ros-kinetic-desktop-full
```
6. Initialize `rosdep`
```
$ sudo rosdep init
$ rosdep update
```
7. Create your workspace:
```
$ mkdir ~/catkin_ws/src/
$ cd ~/catkin_ws
$ catkin_make
```
8. source your new setup using the command:
```
$ source devel/setup.bash
```
You'll need to source your .bash file every time you open a new shell. Save some time by making an alias for this command in your .bash_aliases file.

## Part 3: Download the `rospicam_node` and `luggo` packages.
1. In your `src` directory of `/catkin_ws`, run the commands:
```
$ git clone https://github.com/dganbold/raspicam_node.git
$ git clone https://github.com/aamirhatim/luggo.git
```
2. Run `catkin_make` in your workspace:
```
$ cd ~/catkin_ws
$ catkin_make
```

## Part 4: Launch
