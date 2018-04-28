# Puck Info (puck_info)

## Description

Puck Info is responsible for giving information about the biggest Puck visible.
It gives the color of the Puck, the coordinates (x, y) of the Puck's centroid and if the claw has the Puck.

## Message

Puck Info has its own message type. **PuckInfoMsg**.

PuckInfoMsg header file can be included with `#include "puck_info/PuckInfoMsg.h"`

It's composed by:

- bool has_puck
- geometry_msgs/Point center
- int64 color

## Dependencies

Puck Info depends on the following packages:

- roscpp
- std_msgs
- cv_bridge 		(for OpenCV image conversion)
- geometry_msgs 	(for __center__ informatio)
- sensor_msgs 		(for the robot sensor readings)
- genmsg		(for generating PuckInfoMsg)
- image_transport	(for generating PuckInfoMsg)
- message_generation	(for generating PuckInfoMsg)

