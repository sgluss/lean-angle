# Getting started
Prereqs: install Eigen for Arduino as follows:
Follow the instructions here to get stlPort for Arduino:
https://forum.arduino.cc/index.php?topic=576433.0
use this Arduino port of Eigen:
https://github.com/vancegroup/EigenArduino

To run tests:
`make test`
`make clean`

# System calibration
To determine device orientation, we must find sufficient information to build a fully constrained orientation. For this system, we will use a down vector (up) and a lean vector (device in same position, but leaned to the left side).

This program uses JPL notation for quaternions: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Definition

## MPU System Calibration
Occurs in setup() function on system start. Requires no user action.

## Initiate orientation calibration
Triggered by pushbutton, left led will light, brief pause before calibration begins

## Down vector determination
Device must be held vertically upright, first LED will illuminate.

down vector is determined by averaging accelerometer readings.

Once down vector is measured, the second (middle) LED will light. This indicates the system is ready to find the leaned reference.

## Leaned Reference
Cross product is used to find orthogonal vector components of the rotation matrix: https://en.wikipedia.org/wiki/Cross_product

The measured vector with greatest confidence will be our down vector. The forward vector is only really needed to determine yaw orientation.

Lean the system to the LEFT side. Once sufficient lean angle is reached, the 3rd LED will light.

The system will measure the gravity vector at lean, and the system will return to the running state as soon as enough data is collected.

The cross product of the down vector with this leaned vector will produce the forward vector, and the cross product of the down and forward vectors will produce the right vector. 

# Correcting the AHRS quaternion output for device orientation
To achieve this we must produce a rotation quaternion from three orthogonal orientation vectors (right, up, forward)

## Create the rotation matrix
Now we have three orthogonal vectors (right, down, forward projection) to build a rotation matrix as follows: 
https://stackoverflow.com/questions/18558910/direction-vector-to-rotation-matrix

## Convert the rotation matrix into a rotation quaternion
The rotation matrix produce the quaternion as follows: https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/

## Finally, apply the rotation quaternion to the AHRS output quaternion
https://robotics.stackexchange.com/questions/15264/how-to-rotate-a-rotation-quaternion-in-the-body-frame-to-a-rotation-quaternion-i

# System output
The system will output an analog signal indicating:
roll angle on pin A6 (0-5v)
pitch angle on pin A5 (0-5v)
