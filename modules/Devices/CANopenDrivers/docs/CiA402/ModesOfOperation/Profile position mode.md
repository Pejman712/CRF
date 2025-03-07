@ingroup group_ppm

### Mode of operation - PPM

This mode can be divided into 2 parts : trajectory generator and position control function.
The target position is the input of the trajectory generator with the limits, velocity, acceleration and deceleration.
Output of the trajectory generator is position demand value (it can also be internal value) which is the input of the position control function. 
The mode is very versitale, (like the PVM mode) and according to the CiA 402 standard there can be 4 types of profiles with specific ramps :

1. linear
2. sin^2
3. jerk free
4. jerk limited

The types of profiles are configured inside the JSON files of the motors. Not all motors have all the profile types implemented.
Check with the documentation of the motor which profile types are supported.

The profile position mode uses the controlword for activating the movement of the motor. The bits that are used for activation of the mode are:

* Bit 4 is used for assuming to new target position.
* Bit 5 is used for changing immediately the target position. (bit 4 and 5 need to be active for this to happen)
* Bit 6 is used for defining if the position is relative or absolute
* Bit 8 is used for stopping/moving the motor (0 - motor is in movement, 1 - motor comes into halt)
* Bit 9 is used for setting a new target position immediately after the previous one reached (bit 4 and 9 need to be active for this to happen)

### Implementation of PPM

Function setProfilePosition(double pos, double vel, double acc, double dec, PositionReference reference) from ICiA402Driver, configures the registers for the profile position mode.

The function accepts five parameters:

* pos
* vel
* acc
* dec
* reference

The pos is the target position of the motor, vel is the maximum velocity of the profile type for this mode, acc and dec are acceleration and deceleration of the motor accordingly, refernce is parameter for setting the type of position : absolute or relative.
For this mode to work pos, vel, acc and dec need to be set.

Inside the function, the mode is configured through these steps:

1. checking if the motor is initialized
2. if the motor is initialized the mode of operation is set and the configuring registers are set, else the function returns an error code.
3. the target registers are set (commanding parameters)
4. the controlword is actiavted to move the motor

If the function passes through all the parts it returns true, else returns an error code.

For more information about the mode check CiA 402 documentation.

Parameters:

* pos - target position
* vel - maximum velocity of the profile type used in this mode
* acc - acceleration of the motor
* dec - deceleration od the motor
* reference - parameter for setting the position either to absolute or relative

Retruns: True if setting the mode was successful

Returns: Error code if setting the mode was unsuccessful
