#include <fast_tf/fast_tf.hpp>
#include <benchmark/benchmark.h>

// ros-includes
#include <ros/time.h>
#include <tf2/time_cache.h>
#include <tf2/transform_storage.h>

#include <chrono>
#include <thread>

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
  for (auto _ : _state) {
    // setup the structure
    tf2::TimeCache cache(ros::Duration(0.1));

    // write the data into the cache
    for (size_t ii = 0; ii != steps; ++ii) {
      data.stamp_ = ros::Time::now();
      if (!cache.insertData(data, &err))
        std::cout << "failed to insert: " << err << std::endl;
    }
  }
}

BENCHMARK_DEFINE_F(timed_sequence_fixture, write)(State& _state) {
  for (auto _ : _state) {
    // setup the structure
    timed_sequence cache(ros::Duration(0.1));

    // write the data into the cache
    for (size_t ii = 0; ii != steps; ++ii)
      cache.insert(ros::Time::now(), data);
  }
}

// benchmark checks the performance for in-order writing without pruning for
// the proposed and the default tf2 implementation.
BENCHMARK_REGISTER_F(tf_fixture, write)->Range(10, 1000);
BENCHMARK_REGISTER_F(timed_sequence_fixture, write)->Range(10, 1000);

BENCHMARK_DEFINE_F(tf_fixture, random_write)(State& _state) {
  for (auto _ : _state) {
    // setup the structure
    tf2::TimeCache cache(ros::Duration(0.1));

    // simulate noisy data (adding information in between)
    const ros::Time now(ros::Time::now());
    ros::Duration dur;
    dur.nsec = steps;
    bool plus = true;

    // write the data into the cache
    for (size_t ii = 0; ii != steps; ++ii, --dur.nsec) {
      // alter the time-stamp
      data.stamp_ = plus ? now + dur : now - dur;
      plus = !plus;

      if (!cache.insertData(data, &err))
        std::cout << "failed to insert: " << err << std::endl;
    }
  }
}

BENCHMARK_DEFINE_F(timed_sequence_fixture, random_write)(State& _state) {
  for (auto _ : _state) {
    // setup the structure
    timed_sequence cache(ros::Duration(0.1));

    // simulate noisy data (adding information in between)
    const ros::Time now(ros::Time::now());
    ros::Duration dur;
    dur.nsec = steps;
    bool plus = true;

    // write the data into the cache
    for (size_t ii = 0; ii != steps; ++ii, --dur.nsec) {
      // alter the time-stamp
      ros::Time stamp = plus ? now + dur : now - dur;
      plus = !plus;

      cache.insert(stamp, data);
    }
  }
}

// benchmark checks the performance for out-of-order writing without pruning for
// the proposed and the default tf2 implementation.
BENCHMARK_REGISTER_F(tf_fixture, random_write)->Range(10, 1000);
BENCHMARK_REGISTER_F(timed_sequence_fixture, random_write)->Range(10, 1000);

BENCHMARK_DEFINE_F(tf_fixture, prune_all)(State& _state) {
  tf2::TimeCache cache(ros::Duration(0.1));
  ros::Duration dur;
  const ros::Time now(ros::Time::now() - ros::Duration(0.1));

  // generate some data
  for (size_t ii = 0; ii != steps; ++ii) {
    dur.nsec = ii;
    data.stamp_ = now + dur;

    if (!cache.insertData(data, &err))
      std::cout << "failed to insert: " << err << std::endl;
  }

  for (auto _ : _state) {
    _state.PauseTiming();
    // setup the structure
    tf2::TimeCache curr_cache = cache;
    _state.ResumeTiming();
    // drop all data
    data.stamp_ = ros::Time::now() + ros::Duration(0.1);
    if (!curr_cache.insertData(data, &err))
      std::cout << "failed to insert: " << err << std::endl;
  }
}

BENCHMARK_DEFINE_F(timed_sequence_fixture, prune_all)(State& _state) {
  timed_sequence cache(ros::Duration(0.1));
  ros::Duration dur;
  const ros::Time now(ros::Time::now() - ros::Duration(0.1));

  // generate some data
  for (size_t ii = 0; ii != steps; ++ii) {
    dur.nsec = ii;
    cache.insert(now + dur, data);
  }

  for (auto _ : _state) {
    // setup the structure
    _state.PauseTiming();
    timed_sequence curr_cache = cache;
    _state.ResumeTiming();
    curr_cache.insert(ros::Time::now() + ros::Duration(0.1), data);
  }
}

// benchmarks check the performance in pruning the old data.
BENCHMARK_REGISTER_F(tf_fixture, prune_all)->Range(10, 1000);
BENCHMARK_REGISTER_F(timed_sequence_fixture, prune_all)->Range(10, 1000);

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