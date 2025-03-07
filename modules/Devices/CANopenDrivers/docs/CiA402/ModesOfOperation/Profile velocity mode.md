@ingroup group_pvm

### Mode of operation - PVM

This mode can be divided into 4 parts (or sub-functions) :

* demand value input via trajectory generator
* velocity capture using position sensor or velocity sensor
* velocity control function with appropriate input and output signals
* monitoring of the profile velocity using a window-function
* monitoring of velocity actual value using a threshold.

The target velocity is the input for the trajectory generator along with the limits, acceleration and deceleration. Output of the trajectory generator is the velocity demand value which is the input for the velocity controller.

The profile vlocity mode is very versitale, and according to the CiA 402 standard there can be 4 types of profiles with specific ramps :

1. linear
2. sin^2
3. jerk free
4. jerk limited

The types of profiles are configured inside the JSON files of the motors. Not all motors have all the profile types implemented. Anyone who uses this mode should check with the documentation of the motor which profile types are supported.

The profile velocity mode uses the controlword for activating the movement of the motor. The bits that are used for activation of the mode are:

The controlword specifically uses 1 bit for this mode.
Bit 8 is used for stopping/moving the motor (0 - motor is in movement, 1 - motor comes into halt).


### Implementation of PVM

Function setProfileVelocity(double vel, double acc, double dec), from ICiA02Driver, configures the registers for the profile velocity mode. 

The function accepts three parameters:

* vel
* acc
* dec 

The vel is the desired velocity for the profile velocity of the motor, (or the target/max velocity of the profile), whilst acc and dec are the acceleration and deceleration of the motor for the profile velocity mode. 
All 3 parameters are commanding parameters and need to be set for the mode to work.

Inside the function, the mode is configured through these steps:

1. checking if the motor is initialized
2. if the motor is initialized the mode of operation is set and the configuring registers are set, else the function returns an error code.
3. the target registers are set (commanding parameters)
4. the controlword is actiavted to move the motor

If the function passes through all the parts it returns true, else returns an error code.

For more information about the mode check CiA 402 documentation.

Parameters:

* vel- desired (target) velocity for the profile velocity
* acc - acceleration of the motor for the velocity profile
* dec - deceleration of the motor for the velocity profile

Returns: True if the setting of the mode was successful

Returns: Error code if setting the mode was unsuccessful
