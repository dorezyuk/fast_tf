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
/// Class stores a single transform (our version of tf2::StaticCache).
struct static_transform {
  static_transform() = default;
  explicit static_transform(const Eigen::Isometry3d& _data) noexcept;

  void
  set(const Eigen::Isometry3d& _data) noexcept;

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
 * assert(c && "counter is empty");
 *
 * // create a scoped counter. counter is now at 1
 * scoped_handle sc1{c};
 * assert(!c && "counter is now incremented");
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
/// implementation matches the implementation of the tf2::TimedCache.
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
  // the std::map performs better than boost::container::flat_map or std::deque
  // for our usecase.
  using impl_t = std::map<time_t, Eigen::Isometry3d>;
  impl_t map_;  ///< impl holding the data
};

// our transforms may be either static or dynamic.
using variable_transform = std::variant<static_transform, dynamic_transform>;

/// @brief Structure holding a transformation tree.
///
/// The tree is a cycle-tree, connected graph, where every node has one parent
/// (but the root), and an arbitrary number of children.
struct transform_tree {
  /// @brief Structure holding the transform.
  struct node {
    // constructors which will set depth and parent automatically
    explicit node(variable_transform&& _data) noexcept;
    node(variable_transform&& _data, const node& _parent) noexcept;

    unsigned int depth = 0;        ///< the current depth (increasing from root)
    const node* parent = nullptr;  ///< link to the parent; we don't own it
    variable_transform data;       ///< the actual data; we own it
  };

  // the data structure mapping frame-id to the node
  using impl_t = std::unordered_map<std::string, node>;

  // the iterators
  using iterator = impl_t::iterator;
  using const_iterator = impl_t::const_iterator;
  using pair_type = std::pair<iterator, bool>;

  // key-value types
  using value_type = impl_t::value_type;  // node
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
  /// @return Pair of an iterator to the root node and boolean if the new node
  /// has been created (same signature as std::unordered_map::emplace).
  ///
  /// @throw std::runtime_error if new root node can't be inserted in the tree.
  [[nodiscard]] pair_type
  emplace_parent(const std::string& _parent_frame,
                 const std::string& _child_frame, bool _is_static);

  /// @brief Returns the iterator to the node identified by _child_frame.
  ///
  /// The function will create a new node, if the node does not exist. It may
  /// also create a new root node, if the parent node does not exist (see
  /// emplace_parent).
  ///
  /// @return Pair of an iterator to the child node and a boolean if a new node
  /// (child or parent) has been created.
  ///
  /// @throw std::runtime_error if a circle would be created, or if the tree
  /// would be disconnected.
  [[nodiscard]] pair_type
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
  merge(transform_tree& _other);

  impl_t data_;
  impl_t::iterator root_ = data_.end();
};

/// @brief Thread-safe structure allowing to store and query transforms.
struct transform_buffer {
  transform_buffer();

  void
  set(const geometry_msgs::TransformStamped& _tf, bool _is_static);

  geometry_msgs::TransformStamped
  get(const std::string& _target, const std::string& _source,
      const ros::Time& _time, const ros::Duration& _tolerance);

protected:
  // we may have multiple trees.
  using trees_t = std::list<transform_tree>;

  [[nodiscard]] bool
  same_tree(trees_t::iterator _l, trees_t::iterator _r) const noexcept {
    return _l != trees_.end() && _r != trees_.end() && _l == _r;
  }

  [[nodiscard]] transform_tree::pair_type
  emplace(const std::string& _parent_frame, const std::string& _child_frame,
          bool _is_static);

  [[nodiscard]] std::pair<trees_t::iterator, transform_tree::iterator>
  find(const std::string& _frame);

  void
  merge(const std::string& _parent_frame, const std::string& _child_frame);

  bool
  merge(const std::string& _common_frame);

  // attribute will be ignored on non gcc compilers
  Eigen::Isometry3d
  get_transform(const time_t& _query_time, const time_t& _end_time,
                const transform_tree::node* _node,
                std::unique_lock<std::mutex>& _lock ) __attribute__((nonnull));

  counter c_;
  trees_t trees_;
  std::mutex m_;
  std::condition_variable cv_;
};

}  // namespace detail
}  // namespace fast_tf

#endif  // FAST_TF_FAST_TF_HPP__
