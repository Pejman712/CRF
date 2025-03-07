@ingroup group_ptm

### Mode of operation - PTM

The profile torque mode can be divided into two parts: trajectory generator and the torque control and power stage controller.
The trajectory generator is responsible for the design of the profile type of the torque. It receives values such as target 
torque, target slope, torque profile type, max torque and controlword. The output is the torque demand that goes into the
torque control and power stage controller with the limits for this mode of operation. The controller is directly connected 
to the motor.

In this case the explanation is given for the rotary motor. For the linear motor the description would just be deifned through
force instead of torque.

The profile torque mode uses the controlword for activating the movement of the motor. The bits that arre used for activation of the mode are:

* Bit 8 isused for stopping/moving the motor (0 - motor is in movement, 1 - motor comes into halt)

### Implementation of PTM

Function setProfileTorque(double torque) from ICiA40Driver, configures the registers for the profile torque mode.

The function accepts one parameter:

* tor

The tor is the target torque (commanding parameter) of the motor. All of the other registers are configuring registers for this mode and they are defined inside the JSON file for the motor. For this mode to work just setting the torque is enough.

Inside the function, the mode is configured through these steps:

1. checking if the motor is initialized
2. if the motor is initialized the mode of operation is set and the configuring registers are set, else the function returns an error code.
3. the target register is set (commanding parameter)
4. the controlword is actiavted to move the motor

If the function passes through all the parts it returns true, else returns an error code.

For more information about the mode check CiA 402 documentation.

Parameters:

* tor

Retruns: True if setting the mode was successful

Returns: Error code if setting the mode was unsuccessful
