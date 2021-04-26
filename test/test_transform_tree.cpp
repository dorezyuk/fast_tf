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
  ASSERT_ANY_THROW(tt.emplace("foo", {}, true));
  ASSERT_ANY_THROW(tt.emplace({}, "bar", true));
}

TEST_F(transform_tree_fixture, self_transform) {
  // verifies that we don't accept self-transforms
  ASSERT_ANY_THROW(tt.emplace("foo", "foo", true));
}

TEST_F(transform_tree_fixture, basic) {
  // test verifies that we can create a basic transform and that all members are
  // set properly.
  auto res = tt.emplace("foo", "bar", true);
  ASSERT_TRUE(res.second);
  ASSERT_EQ(res.first->first, std::string{"bar"});
  ASSERT_EQ(res.first->second.depth, 1);

  auto parent = tt.data_.find("foo");
  ASSERT_NE(parent, tt.data_.end());
  ASSERT_EQ(parent, tt.root_);
  ASSERT_EQ(res.first->second.parent, &parent->second);
  ASSERT_EQ(parent->second.depth, 0);
  ASSERT_EQ(parent->second.parent, nullptr);
}

TEST_F(transform_tree_fixture, unconnected) {
  // test verifies that we can detect unconnected structs
  // init the tree with a transform
  auto res = tt.emplace("foo", "bar", true);
  ASSERT_TRUE(res.second);

  // now add something unconnected
  ASSERT_ANY_THROW(tt.emplace("another_foo", "another_bar", true));
}

TEST_F(transform_tree_fixture, detect_new_parent) {
  // verifies that we can detect queries which would change the parent.
  // init the tree with a transform
  auto res = tt.emplace("foo", "bar", true);
  ASSERT_TRUE(res.second);

  // now add something unconnected
  ASSERT_ANY_THROW(tt.emplace("another_parent", "bar", true));
}

TEST_F(transform_tree_fixture, static_dynamic) {
  // verifies the logic that the root must always be static.
  tt.emplace("foo", "bar", false);
  auto& bar = tt.data_.find("bar")->second;
  auto& foo = tt.data_.find("foo")->second;
  ASSERT_EQ(bar.depth, 1);
  ASSERT_EQ(foo.depth, 0);
  ASSERT_EQ(foo.data.index(), 0);
  ASSERT_EQ(tt.root_->first, "foo");

  // update the root and make sure that the new child is dynamic
  tt.emplace("baz", "foo", false);
  ASSERT_EQ(bar.depth, 2);
  ASSERT_EQ(foo.depth, 1);
  ASSERT_EQ(foo.data.index(), 1);
  ASSERT_EQ(tt.root_->first, "baz");
}

/// @brief fixture for performing basic integrity checks on a chain.
struct chain_fixture : public transform_tree_fixture {
  const size_t size = 10;  ///< number of links
};

TEST_F(chain_fixture, static_chain) {
  // test verifies that we can easily extend the tree without any issues (we
  // will add childred)
  for (size_t ii = 0; ii != size; ++ii) {
    auto res = tt.emplace(std::to_string(ii), std::to_string(ii + 1), true);
    ASSERT_TRUE(res.second);
    ASSERT_EQ(res.first->second.depth, ii + 1);
  }
}

TEST_F(chain_fixture, update_root) {
  // test verifies that the root of the tree can get updated
  for (size_t ii = 0; ii != size; ++ii) {
    auto res = tt.emplace(std::to_string(ii + 1), std::to_string(ii), true);
    ASSERT_TRUE(res.second);
    ASSERT_EQ(res.first->second.depth, 1);
    ASSERT_EQ(tt.root_->first, std::to_string(ii + 1));
  }
}

TEST_F(chain_fixture, detect_cycle) {
  // test verifies that we can detect and reject cyclic configurations.
  for (size_t ii = 0; ii != size; ++ii) {
    // extend the tree by adding new nodes.
    auto res = tt.emplace(std::to_string(ii), std::to_string(ii + 1), true);
    ASSERT_TRUE(res.second);

    // try to add a connection to all already existing links
    for (size_t cc = 0; cc != ii + 1; ++cc) {
      ASSERT_ANY_THROW(
          tt.emplace(std::to_string(ii + 1), std::to_string(cc), true));
    }
  }
}

TEST_F(transform_tree_fixture, merge_empty) {
  // verifies that the merging is a noop on empty trees.
  transform_tree other;
  ASSERT_NO_THROW(tt.merge(other));
}

TEST_F(transform_tree_fixture, merge_unconnected) {
  // verifies that we detect unconnected trees and don't merge them
  tt.emplace("foo", "bar", false);
  transform_tree other;
  other.emplace("some", "thing", true);

  // try both ways...
  ASSERT_ANY_THROW(tt.merge(other));
  ASSERT_ANY_THROW(other.merge(tt));

  // if a tree cannot be merged, we will not modify it
  ASSERT_EQ(tt.data_.size(), 2);
  ASSERT_EQ(other.data_.size(), 2);
}

TEST_F(transform_tree_fixture, merge_other_into_this) {
  // verifies that the merging of the other tree into this works fine
  tt.emplace("1", "2", true);
  tt.emplace("2", "3", true);
  transform_tree other;

  other.emplace("3", "4", true);
  other.emplace("3", "5", true);
  other.emplace("5", "6", true);

  auto& i3 = tt.data_.find("3")->second;
  tt.merge(other);

  // check the sizes
  ASSERT_TRUE(other.data_.empty());
  ASSERT_EQ(tt.data_.size(), 6);

  // check the depth
  ASSERT_EQ(tt.data_.find("4")->second.depth, 3);
  ASSERT_EQ(tt.data_.find("5")->second.depth, 3);
  ASSERT_EQ(tt.data_.find("6")->second.depth, 4);

  // check the parents
  ASSERT_EQ(tt.data_.find("4")->second.parent, &i3);
  ASSERT_EQ(tt.data_.find("5")->second.parent, &i3);
}

TEST_F(transform_tree_fixture, merge_this_into_other) {
  // verifies that the merging of this tree into the other works fine. very
  // similar to the test above but still slightly different.
  tt.emplace("1", "2", true);
  tt.emplace("2", "3", true);
  transform_tree other;

  other.emplace("2", "4", true);
  other.emplace("2", "5", true);
  other.emplace("5", "6", true);

  auto& i2 = other.data_.find("2")->second;
  other.merge(tt);

  // check the sizes
  ASSERT_TRUE(tt.data_.empty());
  ASSERT_EQ(other.data_.size(), 6);

  // check the depth
  ASSERT_EQ(other.data_.find("4")->second.depth, 2);
  ASSERT_EQ(other.data_.find("5")->second.depth, 2);
  ASSERT_EQ(other.data_.find("6")->second.depth, 3);

  // check the parents
  ASSERT_EQ(other.data_.find("3")->second.parent, &i2);
  ASSERT_EQ(other.data_.find("4")->second.parent, &i2);
  ASSERT_EQ(other.data_.find("5")->second.parent, &i2);
}
