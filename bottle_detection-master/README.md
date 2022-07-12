# Bottle Detection

This project is a sub-task of vision based bottle opening project.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine.

### Prerequisites

1. Install [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu), on Ubuntu 16.04 or [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) on Ubuntu 18.04.

2. Install [Realsense-ROS](https://github.com/IntelRealSense/realsense-ros)

3. Install [Intel RealSense SDK 2.0](https://github.com/IntelRealSense/librealsense)

4. Install [Darknet-ROS](https://github.com/leggedrobotics/darknet_ros)

### Installing

A step by step series of examples that tell you how to get a development env running

1. download the newest version of this package 

```
$ git clone https://gitlab.ipr.kit.edu/uokad/bottle_detection.git
```

2. compile the whole package in your catkin workspace

```
$ catkin build bottle_detection
```

If the catkin build process failed, you should check out if all the prerequisites are fullfilled.

## Running the tests

Explain how to run the automated tests for this system

### Break down into end to end tests

1. Enable the robot:

```
$ roslaunch vision_based_bottle_opening robot.launch
```
2. Check the temperature offset of force sensor:
```
$ rostopic echo /threshold_filtered
```
* If the forces/torques do not equal to 0 even if there is no load, you should modify the [robot_with_bottle_opener_offset.yaml]() under ../ipr_ur/ipr_ur_bringup/config
* Relaunch the robot after modification of the offset file, check if all the forces and torques value equal to 0
* Repeat the previous steps until forces and torques offset <= 0.05

3. Enable the camera and start the YOLO node:

```
$ roslaunch bottle_detection camera_open.launch
```
4. Enable detection nodes:
```
$ roslaunch bottle_detection bottle_detection.launch
```
5. Launch state machine:
```
$ roslaunch general_state_machine simple_look.launch
```
* Delete the rosparam under /generic_states after execution

## Deployment

Add additional notes about how to deploy this on a live system

## Built With

* [Dropwizard](http://www.dropwizard.io/1.0.2/docs/) - The web framework used
* [Maven](https://maven.apache.org/) - Dependency Management
* [ROME](https://rometools.github.io/rome/) - Used to generate RSS Feeds

## Contributing

Please read [CONTRIBUTING.md](https://gist.github.com/PurpleBooth/b24679402957c63ec426) for details on our code of conduct, and the process for submitting pull requests to us.

## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/your/project/tags). 

## Authors

* **Gen Huang** - *Initial work* 

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* Hat tip to anyone whose code was used
* Inspiration
* etc