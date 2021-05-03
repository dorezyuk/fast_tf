#!/usr/bin/env python3

# Node which will publish a dynamic transform chain. This node is used for
# benchmarking the listener_new and listener_legacy nodes.
#
# Specify the depth of the chain with the "source" parameter. The parameter
# must be a unsigned integer, and defaults to three. Additionally you can set a
# delay (making the benchmark "harder") which will be substracted from the
# time-stamp before sending out the transform. The delay defaults to zero.

import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped


def make_transform(parent_frame: str, child_frame: str) -> TransformStamped:
    """Returns a Transform with the given frames.

    Args:
        parent_frame: Frame id of the parent.
        child_frame: Frame id of the child.
    """
    out = TransformStamped()
    out.header.frame_id = parent_frame
    out.child_frame_id = child_frame
    # some random data
    out.transform.rotation.w = 1
    out.transform.translation.x = 0.1

    return out


if __name__ == '__main__':
    rospy.init_node("publisher")
    # setup some static transforms

    pub = rospy.Publisher("/tf", TFMessage, queue_size=10)
    msg = TFMessage()

    delay = rospy.Duration(rospy.get_param("delay", 0.0))
    source = rospy.get_param("source", 3)
    msg.transforms = [make_transform(str(ii), str(ii + 1)) for ii in range(source)]

    while not rospy.is_shutdown():
        rospy.sleep(rospy.Duration(0.05))
        for m in msg.transforms:
            m.header.stamp = rospy.Time.now() - delay

        pub.publish(msg)
