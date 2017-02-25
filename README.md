# ar_joystick
A ROS virtual joystick based on AR-markers

The ar_joy_node converts a transform between two frames to a sensor_msgs Joy message. 

This can work with e.g. [ar_track_alvar](http://wiki.ros.org/ar_track_alvar) or [lcrs_tf_tools](https://github.com/jhu-lcsr/lcsr_tf_tools)

The process of developing this is documented in a [presentation](https://docs.google.com/presentation/d/1rTx04JtYQd5GLaYxBYi-g1EdqgnkUF6qWrShHxI36cU/edit?usp=sharing)

Dependencies:
sudo apt-get install ros-indigo-usb-cam ros-indigo-ar-track-alvar ros-indigo-teleop-twist-joy ros-indigo-turtlesim

