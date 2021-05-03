#include "listener.hpp"

#include <fast_tf/fast_tf_ros.hpp>

#include <ros/ros.h>

#include <list>
#include <thread>

int
main(int argc, char** argv) {
  ros::init(argc, argv, "listener_new");
  ros::NodeHandle nh;

  // setup the data
  fast_tf_ros::Buffer buffer;
  fast_tf_ros::TransformListener listener(buffer);

  // run the test
  run(buffer);
}
