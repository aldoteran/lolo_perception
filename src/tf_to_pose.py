#!/usr/bin/python
"""
Uses the TF from map->sam/base_link to transform the
estimated pose of the docking station from sam/optical_link->DS
to map->DS.
"""
# TODO: Got hardcoded values everywhere.

import rospy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped

if __name__ == "__main__":
    rospy.init_node("sam_tfm_dock_publisher", anonymous=True)

    # Keeping message type consistent, publishing same
    # pose message as our perception module.
    pose_pub = rospy.Publisher("/sam/perception/docking_station_pose",
                               PoseWithCovarianceStamped, queue_size=10)

    # We're gonna use tf2 since it's more stable than tf.
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Frames are kept constant.
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = "map"
    # FIXME: Covariance needed?

    rate = rospy.Rate(10)
    # Run publisher
    while not rospy.is_shutdown():
        try:
            # Get tfm ^S T_{S/DS}.
            sam_tfm_ds = tf_listener.lookup_transform(
                'map', 'docking_station_enu', rospy.Time(0))

            # Apparently pose_msg uses point and tfm_msg uses vector...
            pose.pose.pose.position.x = sam_tfm_ds.transform.translation.x
            pose.pose.pose.position.y = sam_tfm_ds.transform.translation.y
            pose.pose.pose.position.z = sam_tfm_ds.transform.translation.z
            pose.pose.pose.orientation = sam_tfm_ds.transform.rotation
            pose.header.stamp = rospy.Time.now()

        except:
            rospy.logwarn("Can't find transform: sam to base_link")
            continue

        pose_pub.publish(pose)
        rate.sleep()
