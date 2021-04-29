#include <fast_tf/fast_tf.hpp>
#include <benchmark/benchmark.h>

// ros includes
#include <tf2/buffer_core.h>
#include <tf2/transform_storage.h>

using benchmark::State;
using namespace std::chrono_literals;

/// @brief Fixture providing the default tf2::BufferCore
struct tf_buffer_core_static_chain_fixture : public benchmark::Fixture {
  tf2::BufferCore buffer;

  void
  SetUp(const State& _state) override {
    // create a tree
    geometry_msgs::TransformStamped msg;
    msg.transform.rotation.w = 1;
    for (size_t ii = 0; ii != 100; ++ii) {
      msg.header.frame_id = std::to_string(ii);
      msg.child_frame_id = std::to_string(ii + 1);
      buffer.setTransform(msg, "foo", true);
    }
  }
};

/// @brief Fixture providing our transform_buffer
struct transform_buffer_static_chain_fixture : public benchmark::Fixture {
  fast_tf::detail::transform_buffer buffer;

  void
  SetUp(const State& _state) override {
    // create a tree
    const Eigen::Isometry3d tf;
    const auto now(fast_tf::detail::clock_t::now());
    for (size_t ii = 0; ii != 100; ++ii)
      buffer.set(std::to_string(ii), std::to_string(ii + 1), now, tf, true);
  }
};

BENCHMARK_F(tf_buffer_core_static_chain_fixture, forward)
(State& _state) {
  const auto now = ros::Time::now();
  for (auto _ : _state)
    benchmark::DoNotOptimize(buffer.lookupTransform("1", "100", now));
}

BENCHMARK_F(transform_buffer_static_chain_fixture, forward)
(State& _state) {
  const auto now = fast_tf::detail::clock_t::now();
  const auto timeout = 0ms;
  for (auto _ : _state)
    benchmark::DoNotOptimize(buffer.get("1", "100", now, timeout));
}


BENCHMARK_F(tf_buffer_core_static_chain_fixture, backward)
(State& _state) {
  const auto now = ros::Time::now();
  for (auto _ : _state)
    benchmark::DoNotOptimize(buffer.lookupTransform("100", "99", now));
}

BENCHMARK_F(transform_buffer_static_chain_fixture, backward)
(State& _state) {
  const auto now = fast_tf::detail::clock_t::now();
  const auto timeout = 0ms;
  for (auto _ : _state)
    benchmark::DoNotOptimize(buffer.get("100", "99", now, timeout));
}

