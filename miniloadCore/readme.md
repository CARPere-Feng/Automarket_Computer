# Miniload Core

## Introduction

This is the core libraries for running the miniload of unmanned supermarket.


## How to Build

```
git clone --recurse-submodules git@github.com:CARPere-Feng/Automarket_Computer.git
```

or
```$xslt
git clone git@github.com:CARPere-Feng/Automarket_Computer.git
cd folder
git submodule update --init
```

As this library is a ros2 package, you should source `foxy` before `colcon build`.
```
source /opt/ros/foxy/setup.bash
cd to (Automarket_Computer folder)/
colcon build
source install/setup.bash
```

If `colcon build` failed at above, please first use `colcon build` building `third_party` package.

To use it in the way of `.h and .so`, coping out `install/miniloadcore` folder.

Use it in ros2 package way is recommended.

## Quick Start
### Attention!
- 1  
Every time before power off, miniload should move to the origin!  
Every time before power off, miniload should move to the origin!  
Every time before power off, miniload should move to the origin!
- 2  
**Disable Motor**: cut off the current of motor, and stop motor. So, when motor has velocity, this should not be executed.  
**Stop Motor**: doesn't cut off the current, but stop motor. So, there is always 4A current in A motor.


### Absolute Position Mode
Please refer to the demo executable: `examples/motorFeedback.cpp`.

`motor_feeback` method is thread safe.
### Velocity Mode
Please refer to the demo executable: `examples/vel_control.cpp`.  

Since velocity instruction sent immediately, execute velocity mode should using debug mode.