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

counter::scoped_signal::scoped_signal(const counter& _c) :
    counter_(++_c.counter_) {}

counter::scoped_signal::~scoped_signal() { --counter_; }

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

_transform_tree::_leaf::_leaf(const std::string& _frame_id,
                              variable_transform&& _data) noexcept :
    frame_id(_frame_id), data(std::move(_data)) {}

_transform_tree::_leaf::_leaf(const std::string& _frame_id,
                              const _leaf& _parent,
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

_transform_tree::iterator
_transform_tree::init(const std::string& _parent_frame,
                      const std::string& _child_frame) {
  if (!empty()) {
    // check if the child-frame-id is the current root. if not - we must reject
    // the init-call since we already have a valid root-node.
    assert(root_ != data_.end() && "root cannot be data_.end()");
    if (root_->first != _child_frame)
      throw std::runtime_error("cannot reinit the map");

    // increment the map's depth counters
    for (auto& n : data_)
      ++n.second.depth;

    // create the root node and wire up the connections. note: the root node is
    // always static.
    const auto res =
        data_.emplace(_parent_frame, _leaf{_parent_frame, static_transform{}});

    assert(res.second && "failed to create the root node");

    root_->second.parent = &res.first->second;
    root_ = res.first;
  }
  else {
    const auto res =
        data_.emplace(_parent_frame, _leaf{_parent_frame, static_transform{}});

    assert(res.second && "failed to create the root node");

    root_ = res.first;
  }

  assert(!empty() && "tree cannot be empty when leaving init()");
  return root_;
}

_transform_tree::iterator
_transform_tree::emplace(const std::string& _parent_frame,
                         const std::string& _child_frame, bool _is_static) {
  // check if one of the inputs in empty
  if (_parent_frame.empty() || _child_frame.empty())
    throw std::runtime_error("empty frame detected");

  // check if the input is just self-transform
  if (_parent_frame == _child_frame)
    throw std::runtime_error("self-transform detected");

  auto parent = data_.find(_parent_frame);

  // if the parent is unknown we try to add it to the tree a the new root.
  // this only works if the tree is either empty or if the child link is the
  // current root. in all other cases we will throw.
  if (parent == data_.end())
    parent = init(_parent_frame, _child_frame);

  // check if the child is known
  auto child = data_.find(_child_frame);
  if (child == data_.end()) {
    // create a new node
    auto res = data_.emplace(_child_frame, _leaf{_child_frame, parent->second,
                                                 sequence_factory(_is_static)});
    assert(res.second && "failed to insert new child");
    child = res.first;
  }
  else {
    // make sure that the newly added link is consistent. this will also
    // prevent us from creating cyclic graphs.
    if (child->second.parent != &parent->second)
      throw std::runtime_error("child cannot have multiple parents");
  }
  return child;
}

void
_transform_tree::merge(_transform_tree& _other) {
  // calling this on an empty tree is a noop.
  if (_other.empty() || empty())
    return;

  // cn denotes the connecting link between the trees. the syntax below works
  // only with c++17.
  impl_t::const_iterator cn;
  if (cn = data_.find(_other.root_->first); cn != data_.end()) {
    // merge other into this
    for (auto& p : _other.data_)
      p.second.depth += cn->second.depth;
  }
  else if (cn = _other.data_.find(root_->first); cn != _other.data_.end()) {
    // merge this into other
    for (auto& p : data_)
      p.second.depth += cn->second.depth;
  }
  else
    throw std::runtime_error("cannot merge unconnected trees");

  // rewire the pointers to the root
  for (auto& p : _other.data_)
    if (p.second.parent == &_other.root_->second)
      p.second.parent = &root_->second;

  // pass over the members from _other to this.
  data_.merge(_other.data_);

  // only root shall remain in the _other since this is the connecting link.
  assert(_other.size() == 1 && "duplicated links detected");
  _other.data_.clear();
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

static Eigen::Isometry3d
get_transform(dynamic_transform& _dt, const time_t& _time,
              const time_t _end_time) {}

/// @brief applies the transform stored in _l on _r.
static Eigen::Isometry3d
chain_transform(const variable_transform& _l, const Eigen::Isometry3d& _r,
                const time_t& _time) {
  if (std::holds_alternative<static_transform>(_l))
    return std::get<static_transform>(_l).get() * _r;
  else
    return std::get<dynamic_transform>(_l).get(_time) * _r;
}

void
transform_tree::add(const geometry_msgs::TransformStamped& _tf, bool _static) {
  // get the iterator to the child-node.
  auto child = emplace(_tf.header.frame_id, _tf.child_frame_id, _static);
  const Eigen::Isometry3d tf = tf2::transformToEigen(_tf);

  if (_static)
    std::get<0>(child->second.data).set(tf);
  else {
    // dt is the dynamic transform
    auto& dt = std::get<1>(child->second.data);
    dt.set(to_time(_tf.header.stamp), tf);
    // check if there is someone waiting for this transform
    if (dt)
      cv_.notify_all();
  }
}

geometry_msgs::TransformStamped
transform_tree::get(const std::string& _target, const std::string& _source,
                    const ros::Time& _time, const ros::Duration& _tolerance) {
  // tolerance cannot be negative
  std::unique_lock<std::mutex> l(m_);

  // get the iterators to the source and target frames
  auto s_iter = data_.find(_source);
  if (s_iter == data_.end())
    throw std::runtime_error("unknown source frame " + _source);

  auto t_iter = data_.find(_target);
  if (t_iter == data_.end())
    throw std::runtime_error("unknown target frame " + _target);

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
  const time_t query_time(to_time(_time));
  const duration_t tol(to_duration(_tolerance));
  if (tol < duration_t{})
    throw std::runtime_error("tolerance cannot be negative");

  const auto end_time = clock_t::now() + tol;

  while (target->depth < source->depth) {
    // get the transformation of the source branch for the current depth and
    // advance to the next stage.
    source_root =
        get_transform(source->data, query_time, end_time, l) * source_root;
    source = source->parent;
    assert(source && "tree holds a nullptr");
  }

  // at this point both leaves must have equal depth
  assert(target->depth == source->depth && "target and source depth missmatch");

  // now walk up with both
  while (target->parent != source->parent) {
    source_root =
        get_transform(source->data, query_time, end_time, l) * source_root;
    source = source->parent;

    target_root =
        get_transform(target->data, query_time, end_time, l) * target_root;
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

Eigen::Isometry3d
transform_tree::get_transform(const dynamic_transform& _dyn_tr,
                              const time_t& _query_time,
                              const time_t& _end_time,
                              std::unique_lock<std::mutex>& _lock) {
  counter::scoped_signal signal{_dyn_tr};
  do {
    try {
      return _dyn_tr.get(_query_time);
    }
    catch (const std::runtime_error& _ex) {
      ROS_DEBUG_STREAM_THROTTLE(1, "failed to get the transform");
    }
  } while (cv_.wait_until(_lock, _end_time) != std::cv_status::timeout);
  throw std::runtime_error("no transform available");
}

Eigen::Isometry3d
transform_tree::get_transform(const variable_transform& _var_tr,
                              const time_t& _query_time,
                              const time_t& _end_time,
                              std::unique_lock<std::mutex>& _lock) {
  if (_var_tr.index() == 0)
    return std::get<0>(_var_tr).get();
  else
    return get_transform(std::get<1>(_var_tr), _query_time, _end_time, _lock);
}

}  // namespace detail

void
transform_buffer::set(const geometry_msgs::TransformStamped& _tf,
                      bool _stamped) {}

geometry_msgs::TransformStamped
transform_buffer::get(const std::string& _target, const std::string& _source,
                      const ros::Time& _time, const ros::Duration& _tolerance) {
}

}  // namespace fast_tf