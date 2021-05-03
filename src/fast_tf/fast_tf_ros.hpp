#ifndef FAST_TF_FAST_TF_ROS_HPP__
#define FAST_TF_FAST_TF_ROS_HPP__

#include <fast_tf/fast_tf.hpp>

// IWYU pragma: begin_exports
#include <geometry_msgs/TransformStamped.h>
#include <ros/callback_queue.h>
#include <ros/node_handle.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/buffer_interface.h>
// IWYU pragma: end_exports
// IWYU pragma: no_include "tf2/transform_storage.h"

#include <list>
#include <string>

// forward definitions as suggested by IWYU
namespace ros {
class Duration;
class Time;
}  // namespace ros

namespace fast_tf_ros {

/* Some considrations: the original tf2::TransformListener accepts the
 * tf2::BufferCore as input - the class which we replaced in the fast_tf
 * library. Therefore we have to replate the tf2::TransformListener's API here.
 *
 * Since this file is the file the user should include, we mimick the default
 * ROS-style here to ease the transition.
 */

/// @brief Interface for writing into the buffer.
struct BufferSetterInterface {
  /// @brief When called the buffer should drop all stored data.
  virtual void
  clear() = 0;

  /// @brief Inserts the given data into the buffer.
  ///
  /// @param _msg The transform data.
  /// @param _authority The authority (for debugging).
  /// @param _is_static Flag indicating whether the data is dynamic or static.
  virtual void
  setTransform(const geometry_msgs::TransformStamped& _msg,
               const std::string& _authority, bool _is_static) = 0;
};

/// @brief The buffer class.
///
/// Use this in conjunction with the TransformLister the same way you would use
/// the original classes.
struct Buffer : public BufferSetterInterface, public tf2_ros::BufferInterface {
  /// @brief Drops all dynamic data.
  /// @see BufferSetterInterface::clear.
  void
  clear() override;

  /// @brief Inserts the given data into the buffer.
  /// @see BufferSetterInterface::setTransform.
  void
  setTransform(const geometry_msgs::TransformStamped& _msg,
               const std::string& _authority, bool _is_static) override;

  /// @brief Get the transform between two frames by frame ID.
  /// @see tf2_ros::BufferInterface::lookupTransform.
  geometry_msgs::TransformStamped
  lookupTransform(const std::string& target_frame,
                  const std::string& source_frame, const ros::Time& time,
                  const ros::Duration timeout) const override;

  /// @brief Get the transform between two frames by frame ID.
  /// @see tf2_ros::BufferInterface::lookupTransform.
  geometry_msgs::TransformStamped
  lookupTransform(const std::string& target_frame, const ros::Time& target_time,
                  const std::string& source_frame, const ros::Time& source_time,
                  const std::string& fixed_frame,
                  const ros::Duration timeout) const override;

  /// @brief Test if a transform is possible.
  /// @see tf2_ros::BufferInterface::canTransform.
  bool
  canTransform(const std::string& target_frame, const std::string& source_frame,
               const ros::Time& time, const ros::Duration timeout,
               std::string* errstr = nullptr) const override;

  /// @brief Test if a transform is possible.
  /// @see tf2_ros::BufferInterface::canTransform.
  bool
  canTransform(const std::string& target_frame, const ros::Time& target_time,
               const std::string& source_frame, const ros::Time& source_time,
               const std::string& fixed_frame, const ros::Duration timeout,
               std::string* errstr = nullptr) const override;

private:
  fast_tf::detail::transform_buffer impl_;
};

/**
 * @brief Will subscribe to the tf-topics and write the data into the given
 * buffer.
 *
 * The lifetime this object must not exceed the lifetime of the passed buffer
 * object.
 */
struct TransformListener {
  TransformListener(
      BufferSetterInterface& buffer, bool spin_thread = true,
      ros::TransportHints transport_hints = ros::TransportHints());

  TransformListener(
      BufferSetterInterface& buffer, const ros::NodeHandle& nh,
      bool spin_thread = true,
      ros::TransportHints transport_hints = ros::TransportHints());

private:
  using message_type = ros::MessageEvent<tf2_msgs::TFMessage const>;

  void
  callback(const message_type& msg_evt, bool _is_static);

  BufferSetterInterface& buffer_; ///< this must outlive this object
  ros::TransportHints transport_hints_;
  std::list<ros::Subscriber> subs_;
  ros::CallbackQueue queue_;
  ros::AsyncSpinner spinner_;
};

}  // namespace fast_tf_ros

#endif  // FAST_TF_FAST_TF_ROS_HPP__
