#! /usr/bin/env python

import rospy
import tf
import sensor_msgs

def transform_to_joy(translation, rotation):
    return sensor_msgs.msg.Joy()

if __name__ == "__main__":
    rospy.init_node("ar_joy_node")

    joy_publisher = rospy.Publisher("/joy", sensor_msgs.msg.Joy, queue_size=100)  # Publisher for Joystick messages

    listener = tf.TransformListener()  # Listen to messages on the /tf topic and construct a tranformation tree that can be queried

    rate = rospy.Rate(10.0)  # What frequency runs our node on, roughly

    listener.waitForTransform('/world', '/usb_cam', rospy.Time(0), rospy.Duration(1))

    while not rospy.is_shutdown():  # ctrl-C makes this return True
        try:
            (trans,rot) = listener.lookupTransform('/world', '/usb_cam', rospy.Time(0))  # What is the transformation between these frames at the nearest time
            joy_msg = transform_to_joy(trans, rot)

            joy_publisher.publish(joy_msg)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as tf_exception:
            # Go on even if things break, but at least log them
            rospy.logerr(tf_exception)
            continue

        rate.sleep()  # Make node wait for the next tick at the intended frequency
