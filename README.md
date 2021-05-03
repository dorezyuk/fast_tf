# FastTF

FastTF is a drop-in replacement for the [ros/geometry2](https://github.com/ros/geometry2).

## TL;DR

FastTF implements the interface of `tf2_ros::Buffer` without busy-waiting. This results in faster execution:

<p float="left" align="center">
    <img src="perf/listener_stats.png" alt="arrow" style="height: 300px;"/>
</p>

The chart above shows the CPU consumption for a Buffer/TransformListener pair with multiple threads querying the Buffer. The x-axis shows the number of querying threads (with the ticks 10, 50, 100 and 200). The y-axis shows the CPU usage.

The data was obtained using the perf/publisher.py, perf/listener_new and perf/listener_legacy nodes (these nodes aren't part of the installation). The results where obtained on Ubuntu 20.04, running on a AMD Ryzen 5 PRO 4650U with Radeon Graphics CPU. The compiler used is GCC 9.3.0.

## Features

The project is splitted in fast_tf and fast_tf_ros libraries. The first library depends only on Eigen - allowing it to be easier ported to ROS2, eventually. The second library mimics the interfaces of the tf2_ros library - aiming to ease potential switch.

Following features were added w.r.t. tf2_ros:
 - cycle detection at insertion time.
 - true shortest path computation: a query will evaluate only the nodes which connect the target and source frames.
 - no busy waiting: use condition_variables to wake up a query thread when (relevant) new data becomes available.
 - check if a link is already static/dynamic - preventing the change.

## Requirements

FastTF depends on Eigen and tf2 (tf2, tf2_ros, tf2_msgs and tf2_eigen). The compilation requires C++17.

## How To Use

When you want to try out this library, replace the tf2_ros classes with fast_tf classes. The code from the [c++ transform listener tutorial](http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28C%2B%2B%29) would look then something like this:

```c++

// this replaces tf2_ros/transform_listener.h and tf2_ros/buffer.h
#include <fast_tf/fast_tf_ros.hpp>

// same includes as in the tutorial
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "fast_tf_listener");
  ros::NodeHandle node;
  // turtle-sim related code does not change...
  // the only thing that changes is the namespace here
  fast_tf::Buffer tfBuffer;
  fast_tf::TransformListener tfListener(tfBuffer);

  ros::Rate rate(10.0);
  while (node.ok()){
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform("turtle2", "turtle1", ros::Time::now());
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    rate.sleep();
  }
  return 0;
};
```

## Missing Features

The FastTF library is still in development. Following features aren't supported yet:
- call to get the latest transform with ros::Time(0).
- clearing of the buffer on time-jumps.
- message-filters.