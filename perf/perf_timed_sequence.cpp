#include <fast_tf/fast_tf.hpp>
#include <benchmark/benchmark.h>

// ros-includes
#include <ros/time.h>
#include <tf2/time_cache.h>
#include <tf2/transform_storage.h>

#include <chrono>
#include <thread>

using namespace std::chrono_literals;
using benchmark::Fixture;
using benchmark::State;
using fast_tf::detail::timed_sequence;

/// @brief base fixture for loading the parameric iterations
struct parameter_fixture : public Fixture {
  size_t steps;  ///< number of consecutive operations (writes/reads)

  void
  SetUp(const State& _state) override {
    steps = static_cast<size_t>(_state.range(0));
  }
};

/// @brief fixture for running the benchmark on the original implementation
struct tf_fixture : public parameter_fixture {
  tf2::TransformStorage data;  ///< input dummy data
  std::string err;             ///< error string
};

/// @brief fixture for running benchmarks on the proposed implementation
struct timed_sequence_fixture : public parameter_fixture {
  Eigen::Isometry3d data;
};

BENCHMARK_DEFINE_F(tf_fixture, write)(State& _state) {
  tf2::TimeCache cache(ros::Duration(1));
  data.stamp_ = ros::Time::now();
  const ros::Duration increment(1. / steps);

  for (auto _ : _state) {
    for (size_t ii = 0; ii != steps; ++ii) {
      data.stamp_ += increment;
      if (!cache.insertData(data, &err))
        std::cout << "failed to insert: " << err << std::endl;
    }
  }
}

BENCHMARK_DEFINE_F(timed_sequence_fixture, write)(State& _state) {
  timed_sequence cache(1s);
  auto now = std::chrono::system_clock::now();
  const std::chrono::milliseconds increment(1000 / steps);

  for (auto _ : _state) {
    for (size_t ii = 0; ii != steps; ++ii) {
      now += increment;
      cache.insert(now, data);
    }
  }
}

// benchmark checks the performance for in-order writing for the proposed and
// the default tf2 implementation. this case represents the "good" case of the
// fifo queue, where we only insert data at one end and pop from the other.
// the parameter determines the size of the queue.
BENCHMARK_REGISTER_F(tf_fixture, write)->Range(10, 1000);
BENCHMARK_REGISTER_F(timed_sequence_fixture, write)->Range(10, 1000);

BENCHMARK_DEFINE_F(tf_fixture, random_write)(State& _state) {
  tf2::TimeCache cache(ros::Duration(1));
  data.stamp_ = ros::Time::now();
  const ros::Duration increment(0.5 + 1. / steps);
  const ros::Duration decrement(0.5 + 1.11 / steps / 2);

  bool plus = true;

  for (auto _ : _state) {
    for (size_t ii = 0; ii != steps; ++ii) {
      // alter the time-stamp
      if (plus)
        data.stamp_ += increment;
      else
        data.stamp_ -= decrement;
      plus = !plus;

      if (!cache.insertData(data, &err))
        std::cout << "failed to insert: " << err << std::endl;
    }
  }
}

BENCHMARK_DEFINE_F(timed_sequence_fixture, random_write)(State& _state) {
  timed_sequence cache(1s);
  auto now = std::chrono::system_clock::now();
  const std::chrono::nanoseconds increment(size_t(1e9) / steps + size_t(5e8));
  const std::chrono::nanoseconds decrement(size_t(1e9 * 1.11) / steps / 2 +
                                           size_t(5e8));
  bool plus = true;

  for (auto _ : _state) {
    for (size_t ii = 0; ii != steps; ++ii) {
      // alter the time-stamp
      if (plus)
        now += increment;
      else
        now -= decrement;
      plus = !plus;

      cache.insert(now, data);
    }
  }
}

// benchmark checks the performance for out-of-order writing for the proposed
// and the default tf2 implementation. this case represents the "bad" case: when
// inserting new data we alternate between inserting the data in the middle and
// at the end. the parameter determines the size of the queue.
BENCHMARK_REGISTER_F(tf_fixture, random_write)->Range(10, 1000);
BENCHMARK_REGISTER_F(timed_sequence_fixture, random_write)->Range(10, 1000);

int
main(int argc, char** argv) {
  benchmark::Initialize(&argc, argv);
  if (benchmark::ReportUnrecognizedArguments(argc, argv))
    return 1;

  // init ros::Time
  ros::Time::init();
  benchmark::RunSpecifiedBenchmarks();
  return 0;
}