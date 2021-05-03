from os import stat
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

    msg.transforms = [make_transform(str(ii), str(ii + 1)) for ii in range(3)]

    while not rospy.is_shutdown():
        rospy.sleep(rospy.Duration(0.05))
        for m in msg.transforms:
            m.header.stamp = rospy.Time.now()

        pub.publish(msg)
