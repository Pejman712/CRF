@ingroup group_vom

### Mode of operation - VOM


This mode can be divided into 5 parts (or sub-functions) :

* factor function
* velocity limit function
* ramp function
* velocity control function
* reverse factor function

The factor function accepts the values of parametrs for setting the units of the velocity, target velocity and set-points. Velocity limit function accepts all the limit values for velocity. Ramp function accepts all the values of the parameters for setting up the ramp (acceleration, deceleration etc). Additionally the ramp function accepts the setting of the controlword for the activation of the mode. The formed ramp is the input of the velocity control function. The reverse factor function accepts the output of the velocity control function. There are two of the reverse factor functions in the control system, where one gives out the value of the velocity measurement and the other the demanded velocity.

The velocity mode uses the controlword for activating the movement of the motor. The bits that are used for activation of the mode are:

* Bit 4 is used for enabling ramp
* Bit 5 is used for unlocking the ramp
* Bit 6 is used for following the reference ramp
* Bit 8 is used for stopping/moving the motor (0 - motor is in movement, 1 - motor comes into halt)

The bits 4,5,6 need to be set for the mode to work.

### Implementation of VOM

Function setVelocity(double vel, double deltaSpeedAcc, double deltaTimeAcc, double deltaSpeedDec, double delatTimeDec) from ICiA402Driver,configures the registers for the velocity mode.

The function accepts five parameters:

* vel
* deltaSpeedAcc
* deltaTimeAcc
* deltaSpeedDec
* deltaTimeDec

The vel is desired velocity (or target velocity) of the motor. Parameters deltaSpeedAcc and deltaTimeAcc are used to configure the acceleration of the motor (the acceleration ramp) : acc = deltaSpeedAcc/deltaTimeAcc. Parameters deltaSpeedDec and deltaTimeDec are used to configure the deceleration of the motor (the deceleration ramp) : dec = deltaSpeedDec/deltaTimeDec.

All 5 parameters are commanding parameters and need to be set for the mode to work.

Inside the function, the mode is configured through these steps:

1. checking if the motor is initialized
2. if the motor is initialized the mode of operation is set and the configuring registers are set, else the function returns an error code.
3. the target registers are set (commanding parameters)
4. the controlword is actiavted to move the motor

If the function passes through all the parts it returns true, else returns an error code.

For more information about the mode check CiA 402 documentation.

Parameters:

* vel - desired (target) velocity
* deltaSpeedAcc - delta speed of the acceleration ramp
* deltaTimeAcc - delta time of the acceleration ramp
* deltaSpeedDec - delta speed of the deceleration ramp
* deltaTimeDec - delta time of the deceleration ramp

Returns: True if setting the mode was successful

Returns: Error code if setting the mode was unsuccessful
