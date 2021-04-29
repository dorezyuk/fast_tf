#include <fast_tf/fast_tf.hpp>
#include <benchmark/benchmark.h>

// ros-includes
#include <ros/time.h>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
// the time_cache has a redundant ";" - we ignore this warning
#include <tf2/time_cache.h>
#pragma GCC diagnostic pop
#include <tf2/transform_storage.h>

#include <chrono>

using namespace std::chrono_literals;
using benchmark::Fixture;
using benchmark::State;
using fast_tf::detail::dynamic_transform;

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
struct dynamic_transform_fixture : public parameter_fixture {
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

BENCHMARK_DEFINE_F(dynamic_transform_fixture, write)(State& _state) {
  dynamic_transform cache(1s);
  auto now = std::chrono::system_clock::now();
  const std::chrono::duration<double> _increment(1. / steps);
  const std::chrono::nanoseconds increment(
      std::chrono::duration_cast<std::chrono::nanoseconds>(_increment));

  for (auto _ : _state) {
    for (size_t ii = 0; ii != steps; ++ii) {
      now += increment;
      cache.set(now, data);
    }
  }
}

// benchmark checks the performance for in-order writing for the proposed and
// the default tf2 implementation. this case represents the "good" case of the
// fifo queue, where we only insert data at one end and pop from the other.
// the parameter determines the size of the queue.
BENCHMARK_REGISTER_F(tf_fixture, write)->Range(10, 1000);
BENCHMARK_REGISTER_F(dynamic_transform_fixture, write)->Range(10, 1000);

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

BENCHMARK_DEFINE_F(dynamic_transform_fixture, random_write)(State& _state) {
  dynamic_transform cache(1s);
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

      cache.set(now, data);
    }
  }
}

// benchmark checks the performance for out-of-order writing for the proposed
// and the default tf2 implementation. this case represents the "bad" case: when
// inserting new data we alternate between inserting the data in the middle and
// at the end. the parameter determines the size of the queue.
BENCHMARK_REGISTER_F(tf_fixture, random_write)->Range(10, 1000);
BENCHMARK_REGISTER_F(dynamic_transform_fixture, random_write)->Range(10, 1000);

BENCHMARK_DEFINE_F(tf_fixture, read)(State& _state) {
  tf2::TimeCache cache(ros::Duration(1));
  data.stamp_ = ros::Time::now();
  const ros::Duration increment(1. / steps);

  // populate the data
  for (size_t ii = 0; ii != steps; ++ii) {
    data.stamp_ += increment;
    if (!cache.insertData(data, &err))
      std::cout << "failed to insert: " << err << std::endl;
  }

  tf2::TransformStorage out;
  // the query time is between the latest time point and its predecessor
  data.stamp_ -= (increment * 0.5);
  for (auto _ : _state) {
    benchmark::DoNotOptimize(cache.getData(data.stamp_, out, &err));
  }
}

BENCHMARK_DEFINE_F(dynamic_transform_fixture, read)(State& _state) {
  dynamic_transform cache(1s);
  auto now = std::chrono::system_clock::now();
  const std::chrono::duration<double> _increment(1. / steps);
  const std::chrono::nanoseconds increment(
      std::chrono::duration_cast<std::chrono::nanoseconds>(_increment));

  // populate the data
  for (size_t ii = 0; ii != steps; ++ii) {
    now += increment;
    cache.set(now, data);
  }

  // the query time is between the latest time point and its predecessor
  const std::chrono::duration<double> _decrement(.5 / steps);
  const std::chrono::nanoseconds decrement(
      std::chrono::duration_cast<std::chrono::nanoseconds>(_decrement));
  now -= decrement;
  for (auto _ : _state) {
    benchmark::DoNotOptimize(cache.get(now));
  }
}

// benchmark checks the performance for the getting the latest transformation
// for the proposed and the default tf2 implementation. this case represents the
// "best" (and very unrealistic) case where the query time corresponds to the
// last write time.
BENCHMARK_REGISTER_F(tf_fixture, read)->Range(10, 1000);
BENCHMARK_REGISTER_F(dynamic_transform_fixture, read)->Range(10, 1000);

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
