#include <fast_tf/fast_tf.hpp>
#include <fast_tf/fast_tf_ros.hpp>

// IWYU pragma: no_include "Eigen/src/Geometry/Transform.h"

#include <ros/console.h>
#include <ros/duration.h>
#include <ros/forwards.h>
#include <ros/spinner.h>
#include <ros/subscribe_options.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <ros/transport_hints.h>
#include <std_msgs/Header.h>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.h>
// IWYU pragma: no_include "tf2/transform_storage.h"

#include <boost/bind/arg.hpp>
#include <boost/bind/bind.hpp>
#include <boost/bind/placeholders.hpp>
#include <boost/smart_ptr/make_shared_object.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/type_index/type_index_facade.hpp>

#include <chrono>
#include <list>
#include <ostream>
#include <ratio>
#include <stdexcept>
#include <string>
#include <vector>

namespace fast_tf_ros {

namespace ft = fast_tf::detail;

/// @brief conversion from ros::Time to std::chrono.
static ft::time_t
toTime(const ros::Time& _time) {
  using namespace std::chrono;
  return ft::time_t(seconds(_time.sec)) + nanoseconds(_time.nsec);
}

/// @brief conversion from ros::Duration to std::chrono.
static ft::duration_t
toDuration(const ros::Duration& _dur) {
  using namespace std::chrono;
  return duration_cast<ft::duration_t>(seconds(_dur.sec)) +
         duration_cast<ft::duration_t>(nanoseconds(_dur.nsec));
}

void
Buffer::clear() {
  // todo implement the clearing
}

void
Buffer::setTransform(const geometry_msgs::TransformStamped& _msg,
                     const std::string& _authority, bool _is_static) {
  // convert the data
  const auto stamp = toTime(_msg.header.stamp);
  const auto data = tf2::transformToEigen(_msg);
  try {
    // may throw if the tree would be ill-formed.
    impl_.set(_msg.header.frame_id, _msg.child_frame_id, stamp, data,
              _is_static);
  }
  catch (const std::runtime_error& _ex) {
    std::stringstream ss;
    ss << _ex.what() << "; authority " << _authority;
    throw tf2::TransformException(ss.str());
  }
}

static geometry_msgs::TransformStamped
eigenToTransform(const Eigen::Isometry3d& _data,
                 const std::string& _parent_frame,
                 const std::string& _child_frame, const ros::Time& _stamp) {
  auto tf = tf2::eigenToTransform(_data);
  tf.header.frame_id = _parent_frame;
  tf.header.stamp = _stamp;
  tf.child_frame_id = _child_frame;
  return tf;
}

geometry_msgs::TransformStamped
Buffer::lookupTransform(const std::string& target_frame,
                        const std::string& source_frame, const ros::Time& time,
                        const ros::Duration timeout) const {
  const auto stamp = toTime(time);
  const auto dur = toDuration(timeout);
  try {
    const auto result = impl_.get(target_frame, source_frame, stamp, dur);
    return eigenToTransform(result, target_frame, source_frame, time);
  }
  catch (const std::runtime_error& _ex) {
    std::stringstream ss;
    ss << _ex.what() << " from " << source_frame << " to " << target_frame;
    throw tf2::TransformException(ss.str());
  }
}

geometry_msgs::TransformStamped
Buffer::lookupTransform(const std::string& target_frame,
                        const ros::Time& target_time,
                        const std::string& source_frame,
                        const ros::Time& source_time,
                        const std::string& fixed_frame,
                        const ros::Duration timeout) const {
  if (target_time == source_time)
    return lookupTransform(target_frame, source_frame, source_time, timeout);

  const auto msg_tf1 =
      lookupTransform(fixed_frame, source_frame, source_time, timeout);

  const auto msg_tf2 =
      lookupTransform(target_frame, fixed_frame, target_time, timeout);

  const Eigen::Isometry3d tf1 = tf2::transformToEigen(msg_tf1);
  const Eigen::Isometry3d tf2 = tf2::transformToEigen(msg_tf2);
  return eigenToTransform(tf2 * tf1, target_frame, source_frame, target_time);
}

bool
Buffer::canTransform(const std::string& target_frame,
                     const std::string& source_frame, const ros::Time& time,
                     const ros::Duration timeout, std::string* errstr) const {
  try {
    lookupTransform(target_frame, source_frame, time, timeout);
    return true;
  }
  catch (const std::runtime_error& _ex) {
    if (errstr)
      *errstr = _ex.what();
    return false;
  }
}

bool
Buffer::canTransform(const std::string& target_frame,
                     const ros::Time& target_time,
                     const std::string& source_frame,
                     const ros::Time& source_time,
                     const std::string& fixed_frame,
                     const ros::Duration timeout, std::string* errstr) const {
  try {
    lookupTransform(target_frame, target_time, source_frame, source_time,
                    fixed_frame, timeout);
    return true;
  }
  catch (const std::runtime_error& _ex) {
    if (errstr)
      *errstr = _ex.what();
    return false;
  }
}

TransformListener::TransformListener(BufferSetterInterface& buffer,
                                     bool spin_thread,
                                     ros::TransportHints transport_hints) :
    TransformListener(buffer, ros::NodeHandle{}, spin_thread, transport_hints) {
}

TransformListener::TransformListener(BufferSetterInterface& buffer,
                                     const ros::NodeHandle& _nh,
                                     bool spin_thread,
                                     ros::TransportHints transport_hints) :
    buffer_(buffer), transport_hints_(transport_hints), spinner_(1, &queue_) {
  // todo allow to change these topics..
  std::vector<std::string> dynamic_topics{"/tf"}, static_topics{"/tf_static"};
  using boost::placeholders::_1;

  auto static_cb = boost::bind(&TransformListener::callback, this, _1, true);
  auto dynamic_cb = boost::bind(&TransformListener::callback, this, _1, false);

  // nullptr results in using the global queue. the global queue goes hand in
  // hand in not using a dedicated thread.
  auto queue_ptr = spin_thread ? &queue_ : nullptr;

  // make a copy of the passed handle since the ros::NodeHandle::subscribe call
  // is not const.
  ros::NodeHandle nh(_nh);
  
  // subscribe to the dynamic and static topics.
  for (const auto& topic : dynamic_topics) {
    auto opts = ros::SubscribeOptions::create<tf2_msgs::TFMessage>(
        topic, 100, dynamic_cb, ros::VoidConstPtr(), queue_ptr);

    opts.transport_hints = transport_hints_;
    subs_.emplace_back(nh.subscribe(opts));
  }

  for (const auto& topic : static_topics) {
    auto opts = ros::SubscribeOptions::create<tf2_msgs::TFMessage>(
        topic, 100, static_cb, ros::VoidConstPtr(), queue_ptr);
    subs_.emplace_back(nh.subscribe(opts));
  }

  if (spin_thread)
    spinner_.start();
}

void
TransformListener::callback(const message_type& _msg, bool _is_static) {
  // todo add the check for time-jumps
  const tf2_msgs::TFMessage& msg_in = *(_msg.getConstMessage());
  const auto& authority = _msg.getPublisherName();

  for (const auto& msg : msg_in.transforms) {
    try {
      buffer_.setTransform(msg, authority, _is_static);
    }
    catch (const std::runtime_error& _ex) {
      ROS_ERROR_STREAM("failed to setTransform: " << _ex.what());
    }
  }
}

}  // namespace fast_tf_ros