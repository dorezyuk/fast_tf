#include <fast_tf/fast_tf.hpp>
#include <gtest/gtest.h>

#include <chrono>
#include <thread>

using fast_tf::detail::transform_buffer;
using system_clock = fast_tf::detail::clock_t;
using time_stamp = fast_tf::detail::time_t;
using namespace std::chrono_literals;

/// @brief Basic fixture which will provide the test object
struct transform_buffer_fixture : public ::testing::Test {
  transform_buffer tb;
};

TEST_F(transform_buffer_fixture, negative_tolerance) {
  // verifies that we reject negative tolerances
  ASSERT_ANY_THROW(tb.get("foo", "bar", system_clock::now(), -1ms));
}

TEST_F(transform_buffer_fixture, dynamic_static_constance) {
  // verifies that we cannot change dynamic transforms to static and vice versa.
  const auto now(system_clock::now());
  const Eigen::Isometry3d tf{};
  tb.set("foo", "bar", now, tf, true);
  tb.set("baz", "car", now, tf, false);

  ASSERT_ANY_THROW(tb.set("foo", "bar", now, tf, false));
  ASSERT_ANY_THROW(tb.set("baz", "car", now, tf, true));
}

/// @brief Creates a transform_buffer with unconnected trees
struct unconnected_buffer_fixture : public transform_buffer_fixture {
  unconnected_buffer_fixture() {
    // setup the trees.
    tb.set("foo", "bar", system_clock::now(), Eigen::Isometry3d{}, true);
    tb.set("baz", "car", system_clock::now(), Eigen::Isometry3d{}, true);
  }
};

TEST_F(unconnected_buffer_fixture, not_connected) {
  // verifies that we will get an exception if two trees aren't connected.
  ASSERT_ANY_THROW(tb.get("foo", "car", system_clock::now(), 1ms));
}

TEST_F(unconnected_buffer_fixture, found_connection) {
  // verifies that we will notify the waiters if a connection becomes available
  // spawn the thread
  std::thread t([&]() {
    ASSERT_NO_THROW(tb.get("foo", "car", system_clock::now(), 1s));
  });

  // wait for some time
  std::this_thread::sleep_for(10ms);

  // connect the trees!
  tb.set("bar", "baz", system_clock::now(), Eigen::Isometry3d{}, true);
  if (t.joinable())
    t.join();
}

/// @brief Connects the trees with an outdated dynamic transform
struct timeout_buffer_fixture : public unconnected_buffer_fixture {
  timeout_buffer_fixture() {
    tb.set("bar", "baz", system_clock::now() - 1h, Eigen::Isometry3d{}, false);
  }
};

TEST_F(timeout_buffer_fixture, timeout_connected) {
  // verifies that we will get an exception if the tree has a too old connection
  ASSERT_ANY_THROW(tb.get("foo", "car", system_clock::now(), 1ms));
}

TEST_F(timeout_buffer_fixture, found_connected) {
  // verifies that we will notify the waiters if a connection becomes available
  // spawn the thread
  const auto now(system_clock::now());
  std::thread t([&]() { ASSERT_NO_THROW(tb.get("foo", "car", now, 1s)); });

  // wait for some time
  std::this_thread::sleep_for(10ms);

  // provide a possible connection
  tb.set("bar", "baz", now, Eigen::Isometry3d{}, false);
  if (t.joinable())
    t.join();
}

/// @brief Fixture with some links for regression tests
struct populated_buffer_fixture : transform_buffer_fixture {
  Eigen::Isometry3d root_one, root_two, one_tree, one_four;
  time_stamp now;
  populated_buffer_fixture() {
    root_one = Eigen::Isometry3d{
        Eigen::Translation3d(1, 0, 0) *
        Eigen::AngleAxisd(.25 * M_PI, Eigen::Vector3d::UnitZ())};

    root_two = Eigen::Isometry3d{Eigen::Translation3d(0, 1, 0)};
    one_tree = Eigen::Isometry3d{Eigen::Translation3d(0, 0.5, 0.5)};
    one_four = Eigen::Isometry3d{
        Eigen::Translation3d(2, -0.5, 0) *
        Eigen::AngleAxisd(.5 * M_PI, Eigen::Vector3d::UnitY())};

    // setup some links
    now = system_clock::now();

    tb.set("root", "1", now, root_one, true);
    tb.set("root", "2", now, root_two, true);
    tb.set("1", "3", now, one_tree, true);
    tb.set("1", "4", now, one_four, true);
  }
};

// check the children
TEST_F(populated_buffer_fixture, root_one) {
  ASSERT_EQ(tb.get("root", "1", now, 0ms).matrix(), root_one.matrix());
  ASSERT_EQ(tb.get("1", "root", now, 0ms).matrix(),
            root_one.inverse().matrix());
}

TEST_F(populated_buffer_fixture, root_two) {
  ASSERT_EQ(tb.get("root", "2", now, 0ms).matrix(), root_two.matrix());
  ASSERT_EQ(tb.get("2", "root", now, 0ms).matrix(),
            root_two.inverse().matrix());
}

TEST_F(populated_buffer_fixture, one_tree) {
  ASSERT_EQ(tb.get("1", "3", now, 0ms).matrix(), one_tree.matrix());
  ASSERT_EQ(tb.get("3", "1", now, 0ms).matrix(), one_tree.inverse().matrix());
}

TEST_F(populated_buffer_fixture, one_four) {
  ASSERT_EQ(tb.get("1", "4", now, 0ms).matrix(), one_four.matrix());
  ASSERT_EQ(tb.get("4", "1", now, 0ms).matrix(), one_four.inverse().matrix());
}

// check the siblings
TEST_F(populated_buffer_fixture, tree_four) {
  const Eigen::Isometry3d res = one_tree.inverse() * one_four;
  // account for numerical issues
  const auto diff_34 =
      (tb.get("3", "4", now, 0ms).matrix() - res.matrix()).norm();

  const auto diff_43 =
      (tb.get("4", "3", now, 0ms).matrix() - res.inverse().matrix()).norm();

  ASSERT_NEAR(diff_34, 0, 1e-6);
  ASSERT_NEAR(diff_43, 0, 1e-6);
}

TEST_F(populated_buffer_fixture, one_two) {
  const Eigen::Isometry3d res = root_one.inverse() * root_two;

  // account for numerical issues
  const auto diff_12 =
      (tb.get("1", "2", now, 0ms).matrix() - res.matrix()).norm();

  const auto diff_21 =
      (tb.get("2", "1", now, 0ms).matrix() - res.inverse().matrix()).norm();

  ASSERT_NEAR(diff_12, 0, 1e-6);
  ASSERT_NEAR(diff_21, 0, 1e-6);
}

// check the grand children
TEST_F(populated_buffer_fixture, root_tree) {
  const Eigen::Isometry3d res = root_one * one_tree;
  ASSERT_EQ(tb.get("root", "3", now, 0ms).matrix(), res.matrix());
  ASSERT_EQ(tb.get("3", "root", now, 0ms).matrix(), res.inverse().matrix());
}

TEST_F(populated_buffer_fixture, root_four) {
  const Eigen::Isometry3d res = root_one * one_four;
  ASSERT_EQ(tb.get("root", "4", now, 0ms).matrix(), res.matrix());
  ASSERT_EQ(tb.get("4", "root", now, 0ms).matrix(), res.inverse().matrix());
}

// check the aunt
TEST_F(populated_buffer_fixture, two_tree) {
  const Eigen::Isometry3d res = root_two.inverse() * root_one * one_tree;

  const auto diff_23 =
      (tb.get("2", "3", now, 0ms).matrix() - res.matrix()).norm();

  const auto diff_32 =
      (tb.get("3", "2", now, 0ms).matrix() - res.inverse().matrix()).norm();

  ASSERT_NEAR(diff_23, 0, 1e-6);
  ASSERT_NEAR(diff_32, 0, 1e-6);
}

TEST_F(populated_buffer_fixture, two_four) {
  const Eigen::Isometry3d res = root_two.inverse() * root_one * one_four;

  const auto diff_24 =
      (tb.get("2", "4", now, 0ms).matrix() - res.matrix()).norm();

  const auto diff_42 =
      (tb.get("4", "2", now, 0ms).matrix() - res.inverse().matrix()).norm();

  ASSERT_NEAR(diff_24, 0, 1e-6);
  ASSERT_NEAR(diff_42, 0, 1e-6);
}
