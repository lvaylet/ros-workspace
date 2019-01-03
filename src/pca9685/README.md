# Prerequisites

- Enable I2C with `raspi-config` and reboot. Check the PCA9685 board shows up with `i2cdetect -l` and `i2cdetect -y 1`.
- Install Python package with `sudo -H pip install adafruit-pca9685==1.0.1`

# Build

```
cd catkin_ws
chmod +x src/pca9685/scripts/driver.py
catkin_make
source devel/setup.bash
```

# Run

In three difeerent terminals:

```
roscore
```

```
rosrun pca9685 driver.py
```

```
rostopic pub --once /steering/normalized std_msgs/Float32 "0.0"
rostopic pub --once /steering/normalized std_msgs/Float32 "0.5"
rostopic pub --once /steering/normalized std_msgs/Float32 "0.8"
rostopic pub --once /steering/normalized std_msgs/Float32 "1.0"
rostopic pub --once /steering/normalized std_msgs/Float32 "1.2"
```
