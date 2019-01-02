# Prerequisites

- Ubuntu 18.04 or Ubuntu 16.04
- ROS Melodic (`ros-melodic-desktop-full` recommended) for Ubuntu 18.04 or ROS Kinetic (`ros-kinetic-desktop-full` recommended) for Ubuntu 16.04
- OpenCV 3

# Build

```
cd catkin_ws
catkin_make
source devel/setup.bash
```

# Run

Assuming the video file to convert exists in a `data/` subfolder, convert it to a ROS bag in the same folder with:

```
cd catkin_ws
rosrun video2bag video2bag.py data/webcam.mov data/webcam.bag
```

Then play and view the bag with:

```
rosbag play data/webcam.bag
rqt_image_view
```
