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

static_transform::static_transform(const Eigen::Isometry3d& _data) noexcept :
    data_(_data) {}

inline void
static_transform::set(const Eigen::Isometry3d& _data) noexcept {
  data_ = _data;
}

inline const Eigen::Isometry3d&
static_transform::get() const noexcept {
  return data_;
}

dynamic_transform::dynamic_transform() noexcept :
    dur_(std::chrono::duration_cast<duration_t>(
        std::chrono::milliseconds(500))) {}

dynamic_transform::dynamic_transform(const duration_t& _dur) noexcept :
    dur_(_dur) {
  assert(dur_ >= duration_t(0) && "duration must be positive");
}

void
dynamic_transform::set(const time_t& _time,
                       const Eigen::Isometry3d& _data) noexcept {
  // mostlikely the data will come in in order and emplace_hint will not change
  // the entry if its already present.
  map_.emplace_hint(map_.end(), _time, _data);

  // check that the map is not empty - we need to acces the last valid element
  assert(!map_.empty() && "map cannot be empty");
  // rebalance
  auto lb = map_.lower_bound(std::prev(map_.end())->first - dur_);
  map_.erase(map_.begin(), lb);
}

/// @brief returns the vector interpolation between _l and _r.
/// @param _l left value.
/// @param _r right value.
/// @param _t the scale, must be in the interval [0, 1].
static Eigen::Vector3d
lerp(const Eigen::Vector3d& _l, const Eigen::Vector3d& _r, double _t) noexcept {
  return _l * _t + (1 - _t) * _r;
}

/// @brief returns the transform interpolation between _l and _r.
/// @param _l left value.
/// @param _r right value.
/// @param _t the scale, must be in the interval [0, 1].
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

Eigen::Isometry3d
dynamic_transform::get(const time_t& _time) const {
  // get the first element not smaller then _time
  const auto lb = map_.lower_bound(_time);

  // we will end up in this case if the map_ is either empty or if the stored
  // data is too old
  if (lb == map_.end())
    throw std::runtime_error("no matching data");

  // if the time stamps match exactly, we don't need to interpolate
  if (lb->first == _time)
    return lb->second;
  else {
    // check if we have two points to interpolate
    if (lb == map_.begin())
      throw std::runtime_error("no matching data: extrapolation in the past");
    // get the prev iterator
    const auto lower = std::prev(lb);
    // our count() returns a signed integer. the default duration is
    // nanoseconds. the nanoseconds must cover +/- 292 years. see
    // https://en.cppreference.com/w/cpp/chrono/duration
    const auto ratio = static_cast<double>((_time - lower->first).count()) /
                       (lb->first - lower->first).count();

    return lerp(lower->second, lb->second, ratio);
  }
}

transform_tree::_leaf::_leaf(const std::string& _frame_id,
                             variable_transform&& _data) noexcept :
    frame_id(_frame_id), data(std::move(_data)) {}

transform_tree::_leaf::_leaf(const std::string& _frame_id, const _leaf& _parent,
                             variable_transform&& _data) noexcept :
    frame_id(_frame_id),
    depth(_parent.depth + 1),
    parent(&_parent),
    data(std::move(_data)) {}

/// @brief factory returning either base_sequence or dynamic_transform
/// instances.
static variable_transform
sequence_factory(bool _static) noexcept {
  if (_static)
    return static_transform{};
  else
    return dynamic_transform{};
}

/// @brief conversion from ros::Time to std::chrono.
static time_t
to_time(const ros::Time& _time) {
  using namespace std::chrono;
  return time_t(seconds(_time.sec)) + nanoseconds(_time.nsec);
}

/// @brief conversion from ros::Duration to std::chrono.
static duration_t
to_duration(const ros::Duration& _dur) {
  using namespace std::chrono;
  return duration_cast<duration_t>(seconds(_dur.sec)) +
         duration_cast<duration_t>(nanoseconds(_dur.nsec));
}

/// @brief applies the transform stored in _l on _r.
static Eigen::Isometry3d
chain_transform(const variable_transform& _l, const Eigen::Isometry3d& _r,
                const time_t& _time) noexcept {
  if (std::holds_alternative<static_transform>(_l))
    return std::get<static_transform>(_l).get() * _r;
  else
    return std::get<dynamic_transform>(_l).get(_time) * _r;
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

  if (_static)
    std::get<0>(child->second.data).set(tf2::transformToEigen(_tf));
  else
    std::get<1>(child->second.data)
        .set(to_time(_tf.header.stamp), tf2::transformToEigen(_tf));
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
  Eigen::Isometry3d source_root(Eigen::Isometry3d::Identity());
  Eigen::Isometry3d target_root(Eigen::Isometry3d::Identity());
  // convert to std::chrono
  const time_t time(to_time(_time));

  while (target->depth < source->depth) {
    // get the transformation of the source branch for the current depth and
    // advance to the next stage.
    source_root = chain_transform(source->data, source_root, time);
    source = source->parent;
    assert(source && "tree holds a nullptr");
  }

  // at this point both leaves must have equal depth
  assert(target->depth == source->depth && "target and source depth missmatch");

  // now walk up with both
  while (target->parent != source->parent) {
    source_root = chain_transform(source->data, source_root, time);
    source = source->parent;

    target_root = chain_transform(target->data, target_root, time);
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