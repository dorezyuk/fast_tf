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
#include <map>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <variant>

namespace fast_tf {
namespace detail {

// below the definition of two transform types: static and dynamic. static
// transforms are time-independent; dynamic transforms store a sequence of
// transforms. both classes correspont to the tf2::StaticCache and
// tf2::TimedCache, rectively.

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

/// @brief Storage for a sequence of dynamic transforms.
///
/// Class stores a timed sequence of a dynamically changing transforms. The
/// implementation matches the implementation of the tf2::TimedCache. Similar to
/// its static counterpart, the class is not thread-safe.
struct dynamic_transform {
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

/// @brief structure holding the transform-tree data. the tree is connected and
/// cycle-free (checked at insertion)
struct transform_tree {
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

  // todo test me
  // todo add merge
  // todo add way for possibly connecting the trees
private:
  /// @brief leaf of the tree.
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

  // this is the owning part of the memory.
  using impl_t = std::unordered_map<std::string, _leaf>;

  [[nodiscard]] impl_t::iterator
  init(const geometry_msgs::TransformStamped& _tf);

  [[nodiscard]] impl_t::iterator
  init_root(const geometry_msgs::TransformStamped& _tf) noexcept;

  impl_t data_;
};
}  // namespace detail
}  // namespace fast_tf

#endif  // FAST_TF_FAST_TF_HPP__