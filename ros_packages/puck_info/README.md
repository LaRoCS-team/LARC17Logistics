# Puck Info (puck_info)

## 1. Description

Puck Info is responsible for giving information about the biggest Puck visible.
It gives the color of the Puck, the coordinates (x, y) of the Puck's centroid and if the claw has the Puck.

## 2. Message

Puck Info has its own message type. **PuckInfoMsg**.

PuckInfoMsg header file can be included with `#include "puck_info/PuckInfoMsg.h"`

It's composed by:

- bool has_puck
- geometry_msgs/Point center
- int64 color

## 3. Dependencies

Puck Info depends on the following packages:

- roscpp
- std_msgs
- cv_bridge 		(for OpenCV image conversion)
- geometry_msgs 	(for __center__ information)
- sensor_msgs 		(for the robot sensor readings)
- genmsg		(for generating PuckInfoMsg)
- image_transport	(for generating PuckInfoMsg)
- message_generation	(for generating PuckInfoMsg)

## 4. How to run puck_info

Use `rosrun` to run the node puck_info

```commandline
rosrun puck_info puck_info_node
```

The package puck_info has 2 additional parameters to help debugging and run puck_info
in a VREP simulation.

The parameters __debug__ and __vrep__ have boolean values, you can set them true or false, 
example:

```commandline
rosrun puck_info puck_info_node _debug:=true _vrep:=false
```

Pay attention to the parameter syntax. (_< parameterName >:=< value >)

  ### 4.1. debug parameter

The __debug__ parameter makes some information of the detection to be showed in the STDOUT.
Also it open windows with the image obtained from the camera with a circle marker on the 
centroid of the respective color in the window name.

  ### 4.2. vrep parameter

The __vrep__ parameter makes the image obtained from the Vrep simulator to be flipped, because
Vrep sends the image from the camera flipped.

