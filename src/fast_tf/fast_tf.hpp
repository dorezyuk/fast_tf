#ifndef FAST_TF_FAST_TF_HPP__
#define FAST_TF_FAST_TF_HPP__

// ros
#include <geometry_msgs/TransformStamped.h>
#include <ros/duration.h>
#include <ros/time.h>

#include <Eigen/Geometry>

// std-lib
#include <boost/container/flat_map.hpp>
#include <memory>
#include <string>
#include <unordered_map>

namespace fast_tf {
namespace detail {

/// @brief base-sequence for static data
///
/// implementation is similar to tf2::StaticCache. class stores a single
/// Eigen::Isometry3d transform. the transform can be set with
/// base_sequence::insert and retrieved with base_sequence::closest
struct base_sequence {
  virtual ~base_sequence() = default;
  base_sequence() = default;

  /// @brief stores _data interally.
  virtual void
  insert(const ros::Time& _time, const Eigen::Isometry3d& _data) noexcept;

  /// @brief returns the stored data.
  [[nodiscard]] virtual const Eigen::Isometry3d&
  closest(const ros::Time& _time, const ros::Duration& _tolerance);

protected:
  Eigen::Isometry3d data_;
};

/// @brief sequence of timed data.
///
/// extends of the base_sequence such that it can store a sequence of timed
/// transform. corresponds to the tf2::TimedCache implementation.
struct timed_sequence : public base_sequence {
  /// @brief c'tor with the storage duration
  /// @param _dur positive duration
  explicit timed_sequence(const ros::Duration& _dur) noexcept;
  timed_sequence() noexcept;

  /// @brief inserts new data and prunes stale data
  /// @param _time the time-stamp of the data
  /// @param _data the new data to add
  ///
  /// function will remove data with the time-stamps older then ros::Time::now()
  /// - dur_. note: if the time-stamp is not unique, the newer data will
  /// overwrite the older one.
  void
  insert(const ros::Time& _time,
         const Eigen::Isometry3d& _data) noexcept override;

  /// @brief returns the closest element to the query time.
  /// @param _time the query time.
  /// @param _tolerance the transform tolerance.
  /// @throw std::runtime_error if no data can be retrieved.
  [[nodiscard]] const Eigen::Isometry3d&
  closest(const ros::Time& _time, const ros::Duration& _tolerance) override;

private:
  ros::Duration dur_;  ///< the duration how long to keep the data
  using impl_t = boost::container::flat_map<ros::Duration, Eigen::Isometry3d>;
  impl_t map_;  ///< impl holding the data
};

/// @brief structure holding the transform-tree data. the tree is connected and
/// cycle-free (checked add insertion)
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

  geometry_msgs::TransformStamped
  get(const std::string& _target, const std::string& _source,
      const ros::Time& _time, const ros::Duration& _tolerance);

private:
  /// @brief leaf of the tree.
  ///
  /// the structure implements a single-directed list where every node has one
  /// parent but might have multiple leaves. the data to parent is owned
  /// outside.
  struct _leaf {
    // we will use some polymorphism
    using data_t = std::unique_ptr<base_sequence>;

    // constructors which will set depth and parent automatically
    _leaf(const std::string& _name, data_t&& _data) noexcept;
    _leaf(const std::string&, const _leaf& _parent, data_t&& _data) noexcept;

    std::string frame_id;    ///< the name of the leaf
    unsigned int depth = 0;  ///< the current depth (increasing from root)
    const _leaf* parent = nullptr;  ///< link to the parent
    data_t data;                    ///< the actual data
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