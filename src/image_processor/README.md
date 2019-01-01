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
    └── image_processor
        ├── scripts
        └── src
```

change current directory to `catkin_ws` and run:
```
catkin_make
source devel/setup.bash
```

# Run

Make `image_processor.py` executable with:
```
chmod +x src/image_processor/scripts/image_processor.py
```

Run node with:
```
rosrun image_processor image_processor.py
```

