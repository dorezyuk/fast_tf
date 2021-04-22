#include <fast_tf/fast_tf.hpp>
#include <gtest/gtest.h>
#include <ros/time.h>

#include <chrono>
#include <thread>

using namespace std::chrono_literals;
using fast_tf::detail::timed_sequence;
using testing::Test;

/// @brief fixture for testing timed_sequence
struct timed_sequence_fixture : public Test {
  timed_sequence ts;  ///< tested class

  // some dummy test-data
  const Eigen::Isometry3d data1;
  const Eigen::Isometry3d data2;

  timed_sequence_fixture() :
      data1(Eigen::Translation3d(1, 2, 3)),
      data2(Eigen::Translation3d(3, 2, 1)) {}

  void
  SetUp() {
    // time must be initialized in ros
    ros::Time::init();
  }
};

TEST_F(timed_sequence_fixture, basic) {
  // test verifies the basic class mechanics of inserting and querying data.
  const timed_sequence::time_t now(timed_sequence::clock_t::now());

  // the emtpy case
  ASSERT_ANY_THROW(ts.closest(now, 1s));

  // insert some data
  ts.insert(now, data1);

  // query the data - the result must be what we have inserted, since there is
  // only one data point.
  const auto res = ts.closest(now, 1s);

  ASSERT_EQ(res.matrix(), data1.matrix());

  // check the no-data-points cases
  ASSERT_ANY_THROW(ts.closest(now + 2s, 1s));
  ASSERT_ANY_THROW(ts.closest(now - 2s, 1s));
}

TEST_F(timed_sequence_fixture, no_overwrite) {
  // test verifies the overwrite mechanics: for the same time-stamp the newer
  // added data overwrites the older data.
  const timed_sequence::time_t now(timed_sequence::clock_t::now());

  // insert the data1 and check that this worked
  ts.insert(now, data1);
  ASSERT_EQ(ts.closest(now, 0s).matrix(), data1.matrix());

  // update the data and check again
  ts.insert(now, data2);
  ASSERT_EQ(ts.closest(now, 0s).matrix(), data1.matrix());
}

TEST_F(timed_sequence_fixture, end_points) {
  // test verifies that we return the correct (not interpolated) endpoints.
  const timed_sequence::time_t now(timed_sequence::clock_t::now());

  ts.insert(now, data1);
  ts.insert(now + 100ms, data2);

  ASSERT_EQ(ts.closest(now - 200ms, 500ms).matrix(), data1.matrix());
  ASSERT_EQ(ts.closest(now + 200ms, 500ms).matrix(), data2.matrix());
}

/// @brief parametric fixture for testing different interpolation scenarios
/// the parameters is the time-offset t and the expected transformation.
/// the data1 and data2 points will be inserted with +/- 0.01s around the
/// current time.
struct lerp_fixture
    : public timed_sequence_fixture,
      testing::WithParamInterface<std::pair<int, Eigen::Isometry3d>> {
  const timed_sequence::duration_t delta_time;
  lerp_fixture() : delta_time(10) {}
};

INSTANTIATE_TEST_SUITE_P(
    timed_sequence_fixture, lerp_fixture,
    testing::Values(std::make_pair(0, Eigen::Translation3d(2, 2, 2)),
                    std::make_pair(-5, Eigen::Translation3d(1.5, 2, 2.5)),
                    std::make_pair(5, Eigen::Translation3d(2.5, 2, 1.5))));

TEST_P(lerp_fixture, generic) {
  // test verifies that the interpolation mechanics are correct
  const timed_sequence::time_t now(timed_sequence::clock_t::now());
  ts.insert(now + delta_time, data1);
  ts.insert(now - delta_time, data2);

  const auto param = GetParam();
  const auto res =
      ts.closest(now + timed_sequence::duration_t(param.first), 0s);
  ASSERT_EQ(res.matrix(), param.second.matrix());
}

TEST_F(timed_sequence_fixture, rebalance) {
  // test verifies that the rebalance mechanics work correctly
  const timed_sequence::time_t now(timed_sequence::clock_t::now());
  ts.insert(now, data1);

  ASSERT_NO_THROW(ts.closest(now, 500ms));

  // update the data (this will trigger the rebalancing)
  ts.insert(now + 1s, data1);
  ASSERT_ANY_THROW(ts.closest(now, 500ms));
}
