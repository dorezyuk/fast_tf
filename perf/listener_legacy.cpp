#include "listener.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>

#include <list>
#include <thread>

int
main(int argc, char** argv) {
  ros::init(argc, argv, "legacy_listener");
  ros::NodeHandle nh;

  // setup the data
  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener(buffer);

  run(buffer);
}
