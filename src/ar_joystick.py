#! /usr/bin/env python

import rospy
import tf
import sensor_msgs

from tf import transformations

def transform_to_joy(translation, rotation):
    """Convert a transformation to a joystick message
    :param translation a 3-tuple with the x, y, z value
    :param rotation a 4-tuple with the x, y, z, w values of a quaternion"""

    euler = transformations.euler_from_quaternion(rotation)  # (rx, ry, rz)


    axes = (translation[0], translation[1], (1.5-translation[2])) + euler # concatenate the lists, so our virtual joystick has 6 axes: tx, ty, tz and rx, ry, rz


    return sensor_msgs.msg.Joy(axes=axes)  # We have no real virtual button

if __name__ == "__main__":
    rospy.init_node("ar_joy_node")

    ORIGIN, TARGET = "/usb_cam", "/ar_marker_0"

    joy_publisher = rospy.Publisher("/joy", sensor_msgs.msg.Joy, queue_size=100)  # Publisher for Joystick messages

    listener = tf.TransformListener()  # Listen to messages on the /tf topic and construct a transformation tree that can be queried

    rate = rospy.Rate(10.0)  # What frequency runs our node on, roughly

    listener.waitForTransform(ORIGIN, TARGET, rospy.Time(0), rospy.Duration(100))

    while not rospy.is_shutdown():  # ctrl-C makes this return True
        try:
	    now = rospy.Time.now() - rospy.Duration(0.15)
            (trans,rot) = listener.lookupTransform(ORIGIN, TARGET, now)  

            joy_msg = transform_to_joy(trans, rot)
	    joy_msg.buttons = [True]
            joy_publisher.publish(joy_msg)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as tf_exception:
            # Go on even if things break, but at least log them
            # rospy.logerr(tf_exception)
	    joy_msg = sensor_msgs.msg.Joy(axes=[0,0,0,0,0,0], buttons=[False])
	    joy_publisher.publish(joy_msg)
            continue

        rate.sleep()  # Make node wait for the next tick at the intended frequency

