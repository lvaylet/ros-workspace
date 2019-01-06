# Prerequisites

- Install Python package with `sudo -H pip install pyserial==3.4`

# Build

```
cd catkin_ws
chmod +x src/arduino_controller/scripts/controller.py
catkin_make
source devel/setup.bash
```

# Run

In three different terminals:

```
roscore
```

```
rosrun arduino_controller controller.py
```

```
rostopic list
rostopic echo /steering/normalized
```
