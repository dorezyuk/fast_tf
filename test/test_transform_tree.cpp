#include <fast_tf/fast_tf.hpp>
#include <gtest/gtest.h>

#include <geometry_msgs/TransformStamped.h>

using fast_tf::detail::transform_tree;

/// @brief basic fixture for resting the transform_tree class
struct transform_tree_fixture : public testing::Test {
  transform_tree tt;                    ///< class under test
  geometry_msgs::TransformStamped msg;  ///< identity transform

  transform_tree_fixture() { msg.transform.rotation.w = 1; }

  void
  SetUp() override {
    ros::Time::init();
  }
};

TEST_F(transform_tree_fixture, empty_frame) {
  // verifies that we don't accept empty frames
  msg.header.frame_id = "foo";
  ASSERT_ANY_THROW(tt.add(msg, true));

  std::swap(msg.child_frame_id, msg.header.frame_id);
  ASSERT_ANY_THROW(tt.add(msg, true));
}

TEST_F(transform_tree_fixture, self_transform) {
  // verifies that we don't accept self-transforms
  msg.header.frame_id = msg.child_frame_id = "foo";
  ASSERT_ANY_THROW(tt.add(msg, true));
}

TEST_F(transform_tree_fixture, unconnected) {
  // test verifies that we can detect unconnected structs
  // init the tree with a transform
  msg.header.frame_id = "foo";
  msg.child_frame_id = "bar";
  ASSERT_NO_THROW(tt.add(msg, true));

  // now add something unconnected
  msg.header.frame_id += "_unconnected";
  msg.child_frame_id += "_unconnected";
  ASSERT_ANY_THROW(tt.add(msg, true));
}

TEST_F(transform_tree_fixture, detect_new_parent) {
  // verifies that we can detect queries which would change the parent.
  // init the tree with a transform
  msg.header.frame_id = "foo";
  msg.child_frame_id = "bar";
  ASSERT_NO_THROW(tt.add(msg, true));

  // now add something unconnected
  msg.header.frame_id = "new_parent!";
  ASSERT_ANY_THROW(tt.add(msg, true));
}

/// @brief fixture for performing basic integrity checks on a chain.
struct chain_fixture : public transform_tree_fixture {
  const size_t size = 10;  ///< number of links

  void
  check() {
    // now query every possible combination in the chain
    for (size_t ii = 0; ii != size; ++ii) {
      const std::string target(std::to_string(ii));
      for (size_t ss = 0; ss != size; ++ss) {
        const std::string source(std::to_string(ss));
        const auto res = tt.get(target, source, {}, {});

        // check the output
        ASSERT_EQ(res.header.frame_id, target);
        ASSERT_EQ(res.child_frame_id, source);
      }
    }
  }
};

TEST_F(chain_fixture, static_chain) {
  // test verifies that we can easily extend the tree without any issues
  for (size_t ii = 0; ii != size; ++ii) {
    msg.header.frame_id = std::to_string(ii);
    msg.child_frame_id = std::to_string(ii + 1);
    ASSERT_NO_THROW(tt.add(msg, true));
  }

  // verify that the we can use the chain
  check();
}

TEST_F(chain_fixture, update_root) {
  // test verifies that the root of the tree can get updated
  for (size_t ii = 0; ii != size; ++ii) {
    msg.header.frame_id = std::to_string(ii + 1);
    msg.child_frame_id = std::to_string(ii);
    ASSERT_NO_THROW(tt.add(msg, true));
  }

  // verify that the we can use the chain
  check();
}

TEST_F(chain_fixture, detect_cycle) {
  // test verifies that we can detect and reject cyclic configurations.
  for (size_t ii = 0; ii != size; ++ii) {
    // extend the tree by adding new nodes.
    msg.header.frame_id = std::to_string(ii);
    msg.child_frame_id = std::to_string(ii + 1);
    ASSERT_NO_THROW(tt.add(msg, true));

    // try to add a connection to all already existing links
    for (size_t cc = 0; cc != ii + 1; ++cc) {
      msg.header.frame_id = std::to_string(ii + 1);
      msg.child_frame_id = std::to_string(cc);
      ASSERT_ANY_THROW(tt.add(msg, true));
    }
  }
}
