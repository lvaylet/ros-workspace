# Prerequisites

Install numpy and scipy, for example with:
```
sudo apt update && sudo apt install -y python-numpy python-scipy
```

Alternatively, install `pip` with `sudo apt install python-pip`. Then install `numpy` and `scipy` with `pip install --user numpy scipy`.

# Build

Given the following folder structure:
```
catkin_ws
└── src
    ├── beginner_tutorials
    │   ├── include
    │   ├── scripts
    │   └── src
    └── face_detector
        ├── scripts
        └── src
```

change current directory to `catkin_ws` and run:
```
catkin_make
source devel/setup.bash
```

# Run

Make `face_detector.py` executable with:
```
chmod +x src/face_detector/scripts/face_detector.py
```

Run node with:
```
rosrun face_detector face_detector.py
```

