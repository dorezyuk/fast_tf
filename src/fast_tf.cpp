#include <fast_tf/fast_tf.hpp>

#include <geometry_msgs/TransformStamped.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/time.h>
#include <std_msgs/Header.h>
#include <tf2_eigen/tf2_eigen.h>

#include <cassert>
#include <iterator>
#include <stdexcept>
#include <utility>

// the __has_builtin is defined under gcc and under clang
// see https://gcc.gnu.org/onlinedocs/cpp/_005f_005fhas_005fbuiltin.html
// see https://clang.llvm.org/docs/LanguageExtensions.html
// in c++20 we can use this without intrinsics:
// see https://en.cppreference.com/w/cpp/language/attributes/likely
#ifndef __has_builtin
#define __has_builtin(x) 0
#endif

#if __has_builtin(__builtin_expect)
#define likely(x) __builtin_expect((x), 1)
#define unlikely(x) __builtin_expect((x), 0)
#else
// if we don't have the feature then there is no better prediction
#define likely(x) x
#define unlikely(x) x
#endif

namespace fast_tf {
namespace detail {

void
base_sequence::insert([[gnu::unused]] const time_t& _time,
                      const Eigen::Isometry3d& _data) noexcept {
  data_ = _data;
}

const Eigen::Isometry3d&
base_sequence::closest([[gnu::unused]] const time_t& _time,
                       [[gnu::unused]] const ros::Duration& _tolerance) {
  return data_;
}

timed_sequence::timed_sequence() noexcept : dur_(500) {}
timed_sequence::timed_sequence(const ros::Duration& _dur) noexcept :
    dur_((int)(_dur.toSec() * 1000)) {
  assert(dur_ >= ros::Duration(0) && "duration must be positive");
}

void
timed_sequence::insert(const time_t& _time,
                       const Eigen::Isometry3d& _data) noexcept {
  auto res = map_.emplace(_time, _data);
  // we replace it with the old data with the newly passed data in case we
  // already have a transform for the given time-stamp.
  if (unlikely(!res.second)) {
    ROS_DEBUG("overwriting old data");
    res.first->second = _data;
  }
  // rebalance
  auto lb = map_.lower_bound(std::prev(map_.end())->first - dur_);
  map_.erase(map_.begin(), lb);
}

/// @brief lerp-implementation which returns the interpolation between _l and _r
/// @param _l left value
/// @param _r right value
/// @param _t the scale, must be in the interval [0, 1]
static Eigen::Vector3d
lerp(const Eigen::Vector3d& _l, const Eigen::Vector3d& _r, double _t) noexcept {
  assert(_t >= 0 && "scale must be bigger than 0");
  assert(_t <= 1 && "scale must be smaller than 1");
  return _l * _t + (1 - _t) * _r;
}

/// @brief lerp-implementation which returns the interpolation between _l and _r
/// @param _l left value
/// @param _r right value
/// @param _t the scale, must be in the interval [0, 1]
static Eigen::Isometry3d
lerp(const Eigen::Isometry3d& _l, const Eigen::Isometry3d& _r,
     double _t) noexcept {
  assert(_t >= 0 && "ratio must be bigger than 0");
  assert(_t <= 1 && "ratio must be smaller than 1");
  const Eigen::Quaterniond ql(_l.rotation());
  const Eigen::Quaterniond qr(_l.rotation());

  return {Eigen::Translation3d{lerp(_l.translation(), _r.translation(), _t)} *
          ql.slerp(_t, qr)};
}

const Eigen::Isometry3d&
timed_sequence::closest(const time_t& _time, const ros::Duration& _tolerance) {
  // no need to search if the map is empty..
  if (map_.empty())
    throw std::runtime_error("empty map");

  // get the first element not smaller then _time
  const auto lb = map_.lower_bound(_time);

  if (lb == map_.begin()) {
    // lb cannot be end, since the map_ is not empty
    assert(lb != map_.end() && "lower_bound cannot be map::end");

    // check if we respect the upper bound of the given interval.
    // if (lb->first > _time + _tolerance)
    //   throw std::runtime_error("no data: extrapolation in the future");

    return lb->second;
  }
  else if (lb == map_.end()) {
    // we must have a valid prev, since the map_ is not empty.
    auto lower = std::prev(lb);

    // check if we respect the lower bound of the given interval.
    // if (lower->first < _time - _tolerance)
    //   throw std::runtime_error("no data: extrapolation in the past");

    return lower->second;
  }
  else {
    // get the prev iterator
    const auto lower = std::prev(lb);
    // const auto ratio =
    //     (_time - lower->first).toSec() / (lb->first - lower->first).toSec();

    // data_ = lerp(lower->second, lb->second, ratio);
    return data_;
  }
}

transform_tree::_leaf::_leaf(const std::string& _frame_id,
                             data_t&& _data) noexcept :
    frame_id(_frame_id), data(std::move(_data)) {}

transform_tree::_leaf::_leaf(const std::string& _frame_id, const _leaf& _parent,
                             data_t&& _data) noexcept :
    frame_id(_frame_id),
    depth(_parent.depth + 1),
    parent(&_parent),
    data(std::move(_data)) {}

static std::unique_ptr<base_sequence>
sequence_factory(bool _static) noexcept {
  return _static ? std::make_unique<base_sequence>()
                 : std::make_unique<timed_sequence>();
}

void
transform_tree::add(const geometry_msgs::TransformStamped& _tf, bool _static) {
  // check if one of the inputs in empty
  if (_tf.child_frame_id.empty() || _tf.header.frame_id.empty())
    throw std::runtime_error("empty frame detected");

  // check if the input is just self-transform
  if (_tf.child_frame_id == _tf.header.frame_id)
    throw std::runtime_error("self-transform detected");

  // check if the parent is known
  auto parent = data_.find(_tf.header.frame_id);

  // if the parent is unknown we try to add it to the tree a the new root. this
  // only works if the tree is either empty or if the child link is the current
  // root. in all other cases we will throw.
  if (parent == data_.end())
    parent = init(_tf);

  // check if the child is known
  auto child = data_.find(_tf.child_frame_id);
  if (child == data_.end()) {
    // create a new node
    auto res = data_.emplace(
        _tf.child_frame_id,
        _leaf{_tf.child_frame_id, parent->second, sequence_factory(_static)});
    assert(res.second && "failed to insert new child");
    child = res.first;
  }
  else {
    // make sure that the newly added link is consistent. this will also prevent
    // us from creating cyclic graphs.
    if (child->second.parent != &parent->second)
      throw std::runtime_error("child cannot have multiple parents");
  }

  // child->second.data->insert(ros::Time(_tf.header.stamp),
  //                            tf2::transformToEigen(_tf));
}

geometry_msgs::TransformStamped
transform_tree::get(const std::string& _target, const std::string& _source,
                    const ros::Time& _time, const ros::Duration& _tolerance) {
  // tolerance cannot be negative
  assert(_tolerance >= ros::Duration(0) && "tolerance cannot be negative");

  // get the iterators to the source and target frames
  auto s_iter = data_.find(_source);
  if (s_iter == data_.end())
    throw std::runtime_error("unknown source frame");

  auto t_iter = data_.find(_target);
  if (t_iter == data_.end())
    throw std::runtime_error("unknown target frame");

  const _leaf* source = &s_iter->second;
  const _leaf* target = &t_iter->second;

  // we checked that hte source and target are valid nodes - so deref is safe.
  // we order source and target such that source has higher depth.
  if (source->depth < target->depth)
    std::swap(source, target);

  // walk up the tree until we have reached the same depth for both
  Eigen::Isometry3d source_root, target_root;
  while (target->depth < source->depth) {
    // get the transformation of the source branch for the current depth and
    // advance to the next stage.
    // source_root = source_root * source->data->closest(_time, _tolerance);
    source = source->parent;
    assert(source && "tree holds a nullptr");
  }

  // at this point both leaves must have equal depth
  assert(target->depth == source->depth && "target and source depth missmatch");

  // now walk up with both
  while (target->parent != source->parent) {
    // source_root = source_root * source->data->closest(_time, _tolerance);
    source = source->parent;

    // target_root = target_root * target->data->closest(_time, _tolerance);
    target = target->parent;

    assert(source && "tree holds a nullptr");
    assert(target && "tree holds a nullptr");
  }

  // todo we must check in which dir the tf is actually required.
  Eigen::Isometry3d source_target = source_root * target_root.inverse();
  auto msg = tf2::eigenToTransform(source_target);
  msg.child_frame_id = _source;
  msg.header.frame_id = _target;
  msg.header.stamp = _time;
  return msg;
}

transform_tree::impl_t::iterator
transform_tree::init(const geometry_msgs::TransformStamped& _tf) {
  if (!data_.empty()) {
    // check if the child-frame-id is the current root. if not - we must reject
    // the init-call since we already have a valid root-node.
    auto maybe_root = data_.find(_tf.child_frame_id);
    if (maybe_root == data_.end() || maybe_root->second.depth != 0)
      throw std::runtime_error("cannot reinit the map");

    // reinit the map's depth counters
    for (auto& n : data_)
      ++n.second.depth;
    // create the root node and wire up the connections
    auto new_root = init_root(_tf);
    maybe_root->second.parent = &new_root->second;
    return new_root;
  }
  else
    return init_root(_tf);
}

transform_tree::impl_t::iterator
transform_tree::init_root(const geometry_msgs::TransformStamped& _tf) noexcept {
  // create the root node
  const auto res = data_.emplace(
      _tf.header.frame_id, _leaf{_tf.header.frame_id, sequence_factory(true)});
  assert(res.second && "failed to create the root node");
  return res.first;
}

}  // namespace detail

}  // namespace fast_tf