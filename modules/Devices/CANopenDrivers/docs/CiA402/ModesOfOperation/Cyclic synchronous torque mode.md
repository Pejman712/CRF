@ingroup group_cst

### Modde of operation - CST

With this mode, the trajectory generator is located in the control device, not in the drive device.
In cyclic synchronous manner, it provides a target torque to the drive device, which performs torque control. The drive device can provide measurements (values) for actual position, velocity and torque to the control device. For more information check out the CiA 402 standard.

This mode is different from the PTM mode since it doesn't use the controlword for activating the movement of the motor. For this mode the parameters need to be sent synchronously via the PDOs. Torque offset is optional.

### Implementation of CST mode

Function setCyclicTorque(double tor, double torOffset) from ICiA402Driver, configures the registers for the cyclic synchronous torque mode. 
The function accepts two parameters:

* tor 
* torOffset

The tor is the desired torque of the CST mode(the torque that is the reference for the torque controller) whilst torOffset is the commanded offset of the driver.

Inside the function, the mode is configured through these steps:

1. checking if the motor is initialized
2. if the motor is initialized the mode of operation is set and the configuring registers are set, else the function returns an error code.
3. the target registers are set (commanding parameters)
4. torque parameter value is sent to the slave via PDO

If the function passes through all the parts it returns true, else returns an error code.

The user has the freedom to use this function as he wants in the main. If in any case the input value of the torque is over the limit, the motor will go into quickstop. Note that when sending values to the slave (in this case the motor) they need to be sent synchronously, that is why the values of the torque should be sent through a loop.

For example:

@code{.cpp}
int t = 0;
while(t < 100) {
    driver.setCyclicTorque(10);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    t++;
}
@endcode

For more information about the mode check CiA 402 documentation.

Parameters:

* tor - target torque for the CST mode
* torOffset - input value for torque feed forward

Returns: True if the setting of the mode was successful

Returns: Error code if setting the mode was unsuccessful
