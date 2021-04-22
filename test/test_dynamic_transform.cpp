#include <fast_tf/fast_tf.hpp>
#include <gtest/gtest.h>
#include <ros/time.h>

#include <chrono>
#include <thread>

using namespace std::chrono_literals;
using system_clock = fast_tf::detail::clock_t;
using fast_tf::detail::duration_t;
using fast_tf::detail::dynamic_transform;
using time_point = fast_tf::detail::time_t;
using testing::Test;

/// @brief fixture for testing dynamic_transform
struct dynamic_transform_fixture : public Test {
  dynamic_transform ts;  ///< tested class

  // some dummy test-data
  const Eigen::Isometry3d data1;
  const Eigen::Isometry3d data2;

  dynamic_transform_fixture() :
      data1(Eigen::Translation3d(1, 2, 3)),
      data2(Eigen::Translation3d(3, 2, 1)) {}

  void
  SetUp() {
    // time must be initialized in ros
    ros::Time::init();
  }
};

TEST_F(dynamic_transform_fixture, basic) {
  // test verifies the basic class mechanics of inserting and querying data.
  const time_point now(system_clock::now());

  // the emtpy case
  ASSERT_ANY_THROW(ts.get(now));

  // insert some data
  ts.set(now, data1);

  // query the data - the result must be what we have inserted, since there is
  // only one data point.
  const auto res = ts.get(now);

  ASSERT_EQ(res.matrix(), data1.matrix());

  // check the no-data-points cases
  ASSERT_ANY_THROW(ts.get(now + 2s));
  ASSERT_ANY_THROW(ts.get(now - 2s));
}

TEST_F(dynamic_transform_fixture, no_overwrite) {
  // test verifies the overwrite mechanics: for the same time-stamp the newer
  // added data overwrites the older data.
  const time_point now(system_clock::now());

  // insert the data1 and check that this worked
  ts.set(now, data1);
  ASSERT_EQ(ts.get(now).matrix(), data1.matrix());

  // update the data and check again
  ts.set(now, data2);
  ASSERT_EQ(ts.get(now).matrix(), data1.matrix());
}

TEST_F(dynamic_transform_fixture, end_points) {
  // test verifies that we return the correct (not interpolated) endpoints.
  const time_point now(system_clock::now());
  const auto then = now + 100ms;

  ts.set(now, data1);
  ts.set(then, data2);

  ASSERT_EQ(ts.get(now).matrix(), data1.matrix());
  ASSERT_EQ(ts.get(then).matrix(), data2.matrix()) << ts.get(then).matrix();
}

/// @brief parametric fixture for testing different interpolation scenarios
/// the parameters is the time-offset t and the expected transformation.
/// the data1 and data2 points will be inserted with +/- 0.01s around the
/// current time.
struct lerp_fixture
    : public dynamic_transform_fixture,
      testing::WithParamInterface<std::pair<int, Eigen::Isometry3d>> {
  const duration_t delta_time;
  lerp_fixture() : delta_time(10) {}
};

INSTANTIATE_TEST_SUITE_P(
    dynamic_transform_fixture, lerp_fixture,
    testing::Values(std::make_pair(0, Eigen::Translation3d(2, 2, 2)),
                    std::make_pair(-5, Eigen::Translation3d(1.5, 2, 2.5)),
                    std::make_pair(5, Eigen::Translation3d(2.5, 2, 1.5))));

TEST_P(lerp_fixture, generic) {
  // test verifies that the interpolation mechanics are correct
  const time_point now(system_clock::now());

  ts.set(now + delta_time, data1);
  ts.set(now - delta_time, data2);

  const auto param = GetParam();
  const auto res = ts.get(now + duration_t(param.first));
  ASSERT_EQ(res.matrix(), param.second.matrix());
}

TEST_F(dynamic_transform_fixture, rebalance) {
  // test verifies that the rebalance mechanics work correctly
  const time_point now(system_clock::now());

  ts.set(now, data1);

  ASSERT_NO_THROW(ts.get(now));

  // update the data (this will trigger the rebalancing)
  ts.set(now + 1s, data1);
  ASSERT_ANY_THROW(ts.get(now));
}
