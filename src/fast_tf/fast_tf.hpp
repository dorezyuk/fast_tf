#ifndef FAST_TF_FAST_TF_HPP__
#define FAST_TF_FAST_TF_HPP__

// ros
#include <geometry_msgs/TransformStamped.h>

// forward declaration of ros-types, since we just need them for the interface
namespace ros {
class Duration;
class Time;
}  // namespace ros

#include <Eigen/Geometry>

// std-lib
#include <chrono>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <unordered_map>
#include <variant>

namespace fast_tf {
namespace detail {

// below the definition of two transform types: static and dynamic: static
// transforms are time-independent; dynamic transforms store a sequence of
// transforms. both classes correspont to the tf2::StaticCache and
// tf2::TimedCache, respectively.

/// @brief Storage for a static transform.
///
/// Class stores a single transform. The class is not thread-safe, to the
/// locking must be done outside.
struct static_transform {
  static_transform() = default;
  explicit static_transform(const Eigen::Isometry3d& _data) noexcept;

  /// @brief Updates the stored transform.
  /// @param _data The data to be stored.
  void
  set(const Eigen::Isometry3d& _data) noexcept;

  /// @brief Returns the stored transform.
  /// @return The currently stored transform.
  const Eigen::Isometry3d&
  get() const noexcept;

protected:
  Eigen::Isometry3d data_;  ///< the data...
};

// replacing the ros::Time with std::chrono::time_point yields in ~100%
// speedup (!) of the dynamic_transform::set function.
// todo clock_t collides with other definitions
using clock_t = std::chrono::system_clock;
using time_t = std::chrono::time_point<clock_t>;
using duration_t = std::chrono::nanoseconds;

/**
 * @brief Class with a scoped counter. The user can count up as long as the
 * scoped_signal remains in the scope. The scoped_signal will count down as soon
 * as it goes out of scope. The class is not thread-safe.
 *
 * Example
 * @code
 * // create an instance of the counter
 * counter c;
 *
 * // create a scoped counter. counter is now at 1
 * scoped_handle sc1{c};
 *
 * {
 *  scoped_handle sc2{c}; // counter is now at 2
 *  scoped_handle sc3{c}; // counter is now at 3
 * }
 * // counter is now back to 1
 * @endcode
 */
struct counter {
  /// @brief When constructed, the class will increment the count and decrement
  /// when destructed.
  struct scoped_signal {
    scoped_signal(const counter& _c);
    ~scoped_signal();

  private:
    size_t& counter_;
  };

  friend scoped_signal;

  explicit operator bool() const noexcept { return counter_ != 0; }

private:
  mutable size_t counter_ = 0;
};

/// @brief Storage for a sequence of dynamic transforms.
///
/// Class stores a timed sequence of a dynamically changing transforms. The
/// implementation matches the implementation of the tf2::TimedCache. Similar to
/// its static counterpart, the class is not thread-safe.
struct dynamic_transform : public counter {
  /// @brief c'tor with the storage duration
  /// @param _dur positive duration
  explicit dynamic_transform(const duration_t& _dur) noexcept;
  dynamic_transform() noexcept;

  /// @brief Inserts new data and prunes stale data.
  /// @param _time The time-stamp of the data.
  /// @param _data The new data to add.
  ///
  /// Function will prune data older then the newest data - storage duration.
  /// The passed _data does not replace the old data if.
  void
  set(const time_t& _time, const Eigen::Isometry3d& _data) noexcept;

  /// @brief Returns the closest element to the query time.
  /// @param _time The query time.
  /// @throw std::runtime_error if no data can be retrieved.
  Eigen::Isometry3d
  get(const time_t& _time) const;

private:
  duration_t dur_;  ///< the duration how long to keep the data

  // remarks: the benchmarking (see perf/perf_dynamic_transform.cpp) showed that
  // the std::map performs better than boost::container::flat_map for our
  // usecase.
  using impl_t = std::map<time_t, Eigen::Isometry3d>;
  impl_t map_;  ///< impl holding the data
};

// in our case the variant way of polymorphism is more performant than the old
// school virtual method. additionally we can give both classes distinct
// interfaces.
using variable_transform = std::variant<static_transform, dynamic_transform>;

/// @brief Structure holding a transformation tree. The tree is a cycle-free
/// connected chain, where every leaf has one parent (but the root), and an
/// arbitrary number of children.
struct _transform_tree {
  /// @brief Structure holding the transform
  ///
  /// the structure implements a single-directed list where every node has one
  /// parent but might have multiple leaves. the data to parent is owned
  /// outside.
  struct _leaf {
    // constructors which will set depth and parent automatically
    _leaf(const std::string& _name, variable_transform&& _data) noexcept;
    _leaf(const std::string&, const _leaf& _parent,
          variable_transform&& _data) noexcept;

    std::string frame_id;    ///< the name of the leaf
    unsigned int depth = 0;  ///< the current depth (increasing from root)
    const _leaf* parent = nullptr;  ///< link to the parent; we don't own it
    variable_transform data;        ///< the actual data; we own it
  };

  // the data structure
  using impl_t = std::unordered_map<std::string, _leaf>;

  // the iterators
  using iterator = impl_t::iterator;
  using const_iterator = impl_t::const_iterator;

  // key-value types
  using value_type = impl_t::value_type;  // _leaf
  using key_type = impl_t::key_type;      // std::string

  /// @brief Returns true, if the tree holds no data.
  bool
  empty() const noexcept {
    assert(data_.end() == root_ && "root must point to the end");
    return data_.empty();
  }

  /// @brief Initializes the new root node.
  ///
  /// A new root node can be created if the current tree is empty or if the
  /// current root node is the direct child of the new root node: Adding the
  /// link {X->A} to the tree {A->B->C} would then generate the tree
  /// {X->A->B->C}.
  ///
  /// @return iterator to the new root node.
  /// @throw std::runtime_error if new root node can't be inserted in the tree.
  [[nodiscard]] iterator
  init(const std::string& _parent_frame, const std::string& _child_frame);

  /// @brief Returns the iterator to the node identified by _child_frame.
  ///
  /// The function will create a new node, if the node does not exist. It may
  /// also create a new root node, if the parent node does not exist (see init).
  ///
  /// @throw std::runtime_error if a circle would be created, or if the tree
  /// would be disconnected.
  [[nodiscard]] iterator
  emplace(const std::string& _parent_frame, const std::string& _child_frame,
          bool _is_static);

  /// @brief Merges the given tree into the current.
  ///
  /// The trees can only be merged if for at least one tree the common node is
  /// the root node: {A->B->C} and {B->D} have the common node B and B is the
  /// root node of the second tree; the resulting tree would be {A->B->{C, D}}.
  /// The function is a noop, if one of the trees is empty.
  ///
  /// @throw std::runtime_error if the trees cannot be merged.
  void
  merge(_transform_tree& _other);

  impl_t data_;
  impl_t::iterator root_ = data_.end();
};

/// @brief structure holding the transform-tree data. the tree is connected and
/// cycle-free (checked at insertion)
struct transform_tree : private _transform_tree {
  /// @brief will add a new transformation to the tree.
  ///
  /// @param _tf the transform data
  /// @param _static flag indicating if the timed aspect of the transform can be
  /// omitted.
  ///
  /// @throw std::runtime_error if the tree would become unconnected: we must
  /// either have an empty tree, or must know either the parent or the child.
  /// the function also throws if the inserted link would result in a circular
  /// graph.
  void
  add(const geometry_msgs::TransformStamped& _tf, bool _static);

  /// @brief retrieves the tr
  geometry_msgs::TransformStamped
  get(const std::string& _target, const std::string& _source,
      const ros::Time& _time, const ros::Duration& _tolerance);

  bool
  exists(const std::string& _frame) const noexcept {
    return data_.find(_frame) != data_.end();
  }

  // todo test me
  // todo add merge
  // todo add way for possibly connecting the trees
private:
  Eigen::Isometry3d
  get_transform(const dynamic_transform& _dyn_tr, const time_t& _query_time,
                const time_t& _end_time, std::unique_lock<std::mutex>& _lock);

  Eigen::Isometry3d
  get_transform(const variable_transform& _var_tr, const time_t& _query_time,
                const time_t& _end_time, std::unique_lock<std::mutex>& _lock);

  std::mutex m_;
  std::condition_variable cv_;
};
}  // namespace detail

struct transform_buffer {
  void
  set(const geometry_msgs::TransformStamped& _tf, bool _stamped);

  geometry_msgs::TransformStamped
  get(const std::string& _target, const std::string& _source,
      const ros::Time& _time, const ros::Duration& _tolerance);
};
}  // namespace fast_tf

#endif  // FAST_TF_FAST_TF_HPP__