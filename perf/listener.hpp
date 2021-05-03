#ifndef FAST_TF_PERF_LISTENER_HPP__
#define FAST_TF_PERF_LISTENER_HPP__

#include <tf2_ros/buffer_interface.h>
#include <ros/ros.h>

#include <list>
#include <thread>
#include <string>

static void
run(const tf2_ros::BufferInterface& _buffer) {
  ros::NodeHandle nh("~");
  std::list<std::thread> threads;
  const size_t n_threads = nh.param("threads", 100);
  const ros::Duration timeout(nh.param("timeout", 0.1));
  const std::string target("0"), source(std::to_string(nh.param("source", 3)));

  for (size_t ii = 0; ii != n_threads; ++ii)
    threads.emplace_back([&]() {
      ROS_INFO("staring thread");
      while (ros::ok()) {
        try {
          _buffer.lookupTransform(target, source, ros::Time::now(), timeout);
        }
        catch (tf2::TransformException& _ex) {
          ROS_ERROR_STREAM("failed to transform " << _ex.what());
        }
      }
      ROS_INFO("stopping thread");
    });

  ros::spin();

  for (auto& thread : threads)
    thread.join();
}

#endif  // FAST_TF_PERF_LISTENER_HPP__
