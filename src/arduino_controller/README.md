# Prerequisites

- By default you will probably be denied access to the serial port at `/dev/ttyACM0`.

    This issue is related to the permissions on `/dev/ttyACM0`:

    ```
    $ ll /dev/ttyACM0
    crw-rw---- 1 root dialout 166, 0 févr. 11  2016 /dev/ttyACM0
    ```

    This issue can be permanently solved by adding yourself to the `dialout` group with:

    ```
    sudo usermod -a -G dialout $USER
    ```

    You will have to logout and log back in before the group change is recognized.

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
