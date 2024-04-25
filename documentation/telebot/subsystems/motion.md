![The design document for the motion system](../../imgs/motion_system_design.png)

**Note:** At the time of writing this documentation, not all nodes are written for the motion subsystem. Specifically, the Upper Safety, Lower Safety, Lower Driver, and Error Handler nodes are not yet implemented.

## High-level Overview
Control Sources provide the goal positions OR goal velocities for each motor on the TeleBot. Multiple control sources exist, and they are selected from by the Multiplexer node based on the "Active Source" topic. The Active Source topic selects from a group of predefined control sources in a configuration file.

Next, the Splitter uses the IDs of the motors and a config file to determine what topics to map them to (Upper or Lower). Upper and Lower Safety seek to ensure that the robot stays in valid positions (i.e. not colliding with itself). The Upper Driver handles port access and reading and writing to the U2D2 that controls the motors on the torso. The Lower Driver handles port access and reading and writing to the motor drivers and the U2D2 on the Base.

The goal of the Error Handler consolidates error messages from both drivers and provides a common format for reporting them.

## Multiplexer

### Configuration
There are two config files for the multiplexer:
```
.ctrlsrcs  // This name is important, DO NOT CHANGE
multiplexer_config.yaml
```

`.ctrlsrcs` path is specified in the `multiplexer_config.yaml` ***(NOTE: Need to implement)***. `multiplexer_config.yaml` should be passed in as a params file when running the node using ROS.

`.ctrlsrcs` is a custom file type that defines the control source groups available to the multiplexer. The format for the file is:
```
<name>: <topic_1> <topic_2> ... <topic_n>
```
*Note: the file might be white space sensitive, not sure*

The multiplexer config file contains 2 string parameters `default_source` and a `control_sources_path` and a list of ints parameter `invert_motor_goal_ids`. Default source denotes which source from your `.ctrlsrcs` file will be used on startup. `invert_motor_goal_ids` denotes motor ids whose goals should be inverted (meaning, multiplied by -1) to maintain understandable control of the TeleBot. *Note: Don't change this unless you know what you're doing*

### Selecting a control source
Control sources are swapped between using the "Active Source" topic. The "Active Source" topic uses a String message containing the name of the control source to switch to.

See Configuration for setting a default control source.

## Upper Driver
*Note: this is a complex node*
### Configuration
The Upper Driver uses a single configuration file:
``` 
driver_upper_config.yaml
```
The `driver_upper_config.yaml` should be passed as a params file when running the Upper Driver node.

The `pro_motors` and `xm_motors` hold the ids of the dynamixel motors on the torso. The `usb_device` holds the serial id of the U2D2 that controls the dynamixels. This value can be found using commands in Ubuntu, and lets the Upper Driver always connect to the correct U2D2. The `usb_baudrate` is preset to work with the dynamixel motors.

### Implementation
The implementation is split into two parts: a ROS2-based `driver_upper.cpp` node and a ROS-agnostic `dynamixel_driver.hpp`.