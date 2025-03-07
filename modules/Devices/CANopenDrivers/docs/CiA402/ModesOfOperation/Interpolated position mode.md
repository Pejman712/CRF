@ingroup group_ipm

### Mode of operation - IPM

This mode can be divided into 2 parts : trajectory generator and position control function.
The target position is the input of the trajectory generator with the limits, velocity, acceleration and deceleration.
Output of the trajectory generator is position demand value (it can also be internal value) which is the input of the position control function. The mode is very felxible since the user of this mode can implement the trajectory that they want. The trajectory of the motor is implemented by sending set-points to the 60C1h register which allows the trajectory generator to interpolate between the set-points.

The interpolated position mode uses the controlword for activating the movement of the motor. The bits that are used for activation of the mode are:

* Bit 4 is used for setting the IPM mode
* Bit 8 is used for stopping/moving the motor (0 - motor is in movement, 1 - motor comes into halt)
For the mode to work both bit 4 and 8 need to be set.

Note: Linear interpolation is supported for all the motors. There are other types of interpolation but they are manufacturer specific. Check with the documentation which added types of interpolation are supported.
For more information check out the CiA 402 standard.

### Implementation of IPM

Function setInterpolatedPosition(double pos, double vel, double acc,
double dec) from ICiA402Driver,configures the registers for the interpolated position mode.
The function accepts four parameters:

* pos
* vel
* acc
* dec

The pos is the target position of the motor, vel is the maximum velocity of the profile type for this mode, acc and dec are acceleration and deceleration of the motor accordingly.

Inside the function, the mode is configured through these steps:

1. checking if the motor is initialized
2. if the motor is initialized the mode of operation is set and the configuring registers are set, else the function returns an error code.
3. the target registers are set (commanding parameters)
4. position parameter value is sent to the slave via PDO
5. the controlword is actiavted to move the motors

If the function passes through all the parts it returns true, else returns an error code.

The user has the freedom to use this function as they want in the main. If in any case the input value of the position is over the limit, the motor will go into quickstop. Be aware that when using this mode of operation, the starting position of this mode should be the current position of the motor. Note that when sending values to the slave (in this case the motor) they need to be sent synchronously, that is why the values of the position should be sent through a loop.

For example:

@code{.cpp}
crf::expected<double> l = driver.getPosition();
if (!l) return;
double i = l.value();
int t = 0;
while (t < 100) {
    driver.setInterpolatedPosition(i,0.08,0.08,0.04);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    i+= 0.05;
    t++;
}
@endcode

For more information about the mode check CiA 402 documentation.

Parameters:

* pos - target position
* vel - maximum velocity of the profile type used in this mode
* acc - acceleration of the motor
* dec - deceleration od the motor

Returns: True if setting the mode was successful

Returns: Error code if setting the mode was unsuccessful
