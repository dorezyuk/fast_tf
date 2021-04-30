#include <fast_tf/fast_tf.hpp>
#include <benchmark/benchmark.h>

// ros includes
#include <tf2/buffer_core.h>
#include <tf2/transform_storage.h>

#include <array>
#include <chrono>

using benchmark::State;
using namespace std::chrono_literals;

/// @brief Fixture providing the default tf2::BufferCore
struct tf_buffer_core_static_chain_fixture : public benchmark::Fixture {
  tf2::BufferCore buffer;
  ros::Time now;
  std::array<std::string, 100> names;

  void
  SetUp(const State& _state) override {
    for (size_t ii = 0; ii != names.size(); ++ii)
      names.at(ii) = std::to_string(ii);

    // create a tree
    geometry_msgs::TransformStamped msg;
    msg.transform.rotation.w = 0.976;
    msg.transform.rotation.x = 0.216;

    for (size_t ii = 0; ii != names.size() - 1; ++ii) {
      msg.header.frame_id = names.at(ii);
      msg.child_frame_id = names.at(ii + 1);
      msg.transform.translation.x = ii;
      buffer.setTransform(msg, "foo", true);
    }
    now = ros::Time::now();
  }
};

/// @brief Fixture providing our transform_buffer
struct transform_buffer_static_chain_fixture : public benchmark::Fixture {
  fast_tf::detail::transform_buffer buffer;
  fast_tf::detail::time_t now;
  std::chrono::nanoseconds timeout = 0ns;

  std::array<std::string, 100> names;

  void
  SetUp(const State& _state) override {
    for (size_t ii = 0; ii != names.size(); ++ii)
      names.at(ii) = std::to_string(ii);

    // create a tree
    Eigen::Isometry3d tf{Eigen::Quaterniond{0.976, 0.216, 0, 0}};
    now = fast_tf::detail::clock_t::now();
    for (size_t ii = 0; ii != names.size() - 1; ++ii) {
      tf.translation().x() = ii;
      buffer.set(names.at(ii), names.at(ii + 1), now, tf, true);
    }
  }
};

BENCHMARK_F(transform_buffer_static_chain_fixture, forward)
(State& _state) {
  for (auto _ : _state)
    for (size_t ii = 0; ii != names.size() - 1; ++ii)
      benchmark::DoNotOptimize(buffer.get(names.at(ii), "99", now, timeout));
}

BENCHMARK_F(transform_buffer_static_chain_fixture, backward)
(State& _state) {
  for (auto _ : _state)
    for (size_t ii = 0; ii != names.size() - 1; ++ii)
      benchmark::DoNotOptimize(buffer.get("99", names.at(ii), now, timeout));
}

BENCHMARK_F(tf_buffer_core_static_chain_fixture, forward)
(State& _state) {
  for (auto _ : _state)
    for (size_t ii = 0; ii != names.size() - 1; ++ii)
      benchmark::DoNotOptimize(buffer.lookupTransform(names.at(ii), "99", now));
}

BENCHMARK_F(tf_buffer_core_static_chain_fixture, backward)
(State& _state) {
  for (auto _ : _state)
    for (size_t ii = 0; ii != names.size() - 1; ++ii)
      benchmark::DoNotOptimize(buffer.lookupTransform("99", names.at(ii), now));
}
