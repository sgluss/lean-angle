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
To determine device orientation, we must find sufficient information to build a fully constrained orientation. For this system, we will use a down vector (gravity) and a forward vector (forwards acceleration).

##MPU System Calibration
Occurs in setup() function on system start

## Initiate orientation calibration
triggered by pushbutton, brief pause before calibration begins

## Gravity vector determination
Device must be held vertically upright, first LED will illuminate.

Gravity vector is determined by averaging accelerometer readings.

One gravity vector is measured, the second (middle) LED will light. This indicates the system is ready to find the forward reference.

## Forward Reference
Propel the device straight forward. The third (right) LED will light as soon as this movement is detected. 

The system will measure the forward acceleration (by removing the gravity component), and the system will return to the running state as soon as enough data is collected.

# Correcting the AHRS quaternion output for device orientation
To achieve this we must produce a rotation quaternion from three orthogonal orientation vectors (right, up, forward)

## Generate the forward vector projection 
Cross product is used to find orthogonal vector components of the rotation matrix: https://en.wikipedia.org/wiki/Cross_product

The measured vector with greatest confidence will be our down (gravity) vector. This vector is inverted (-) to get the up vector.

 The forward vector is only really needed to determine yaw information. We want to project the forward vector onto the up vector's orthogonal plane to produce an orthogonal forward vector.

First, take the cross product of forward and up to arrive at the right vector. (forward) X (up) = (right)

Second, take cross product of up and right to arrive at the forward projected vector. (up) X (right) = (projected forward)

## Create the rotation matrix
Now we have three orthogonal vectors (right, up, forward projection) to build a rotation matrix as follows: 
https://stackoverflow.com/questions/18558910/direction-vector-to-rotation-matrix

## Convert the rotation matrix into a rotation quaternion
The rotation matrix produce the quaternion as follows: https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/

## Finally, apply the rotation quaternion to the AHRS output quaternion
https://robotics.stackexchange.com/questions/15264/how-to-rotate-a-rotation-quaternion-in-the-body-frame-to-a-rotation-quaternion-i
