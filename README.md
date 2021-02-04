# The Accurate Moving via AMCL for Indoor Service Robot

## Introduction

Although move_base provides a abundent interface for robot moving,
but it doesn't work well in the some senria especially has high requirements of moving accurate.
For example, when robots are doing some tasks about manipulation in front of the desk,
it is supposed to be very closed to the table (maybe 20-30mm).
If using move_base navigation stack, it may lead to plan failure because it shows the collision with the table.
To solve this problems, we propose a noval method for indoor navigation stack via PID and amcl localization package.

## Requirement

The project is based on ROS Kinect, so you need to install ROS first.
Full desktop version is recommanded because some component such as dynamic reconfiguration and rviz is used.

You should put this project as a ros package under the src folder of workspace.

```bash
cd [your_workspace]/src
git clone git@github.com:jiangxkjohn/hit_move.git
```

Then compile this package

```bash
catkin_make -j8 -DCATKIN_WHITELIST_PACKAGES="hit_move"
```
