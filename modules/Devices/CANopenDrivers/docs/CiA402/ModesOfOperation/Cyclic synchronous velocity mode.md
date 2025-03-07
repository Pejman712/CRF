@ingroup group_csv

### Mode of operation - CSV

With this mode, the trajectory generator is located in the control device, not in the drive device. In cyclic synchronous manner, it provides a target velocity to the drive device,  which performs velocity control and torque control (a cascade control system). The drive device can provide measurements (values) for actual position, velocity and torque to the control device. For more information check out the CiA 402 standard.

This mode is different from the PVM and VM mode since it doesn't use the controlword for activating the movement of the motor. For this mode the parameters need to be sent synchronously via the PDOs. Velocity and torque offset are optional.

### Implementation of CSV mode

Function setCyclicVelocity(double vel, double velOffset,
double torOffset) from ICiA402Driver, configures the registers for the cyclic synchronous velocity mode.
The function accepts three parameters:

* vel
* velOffset
* torOffset  

The vel is the velocity of the CSV mode (the velocity that is the reference for the velocity controller) whilst velOffset is commanded offset of the driver and the torOffset is the input value for torque feed forward.

Inside the function, the mode is configured through these steps:

1. checking if the motor is initialized
2. if the motor is initialized the mode of operation is set and the configuring registers are set else the function returns an error code.
3. the target registers are set (commanding parameters)
4. velocity parameter value is sent to the slave via PDO

If the function passes through all the parts it returns true, else returns an error code.

The user has the freedom to use this function as he wants in the main. If in any case the input value of the velocity is over the limit, the motor will go into quickstop. Note that when sending values to the slave (in this case the motor) they need to be sent synchronously, that is why the values of the velocity should be sent through a loop.

For example:

@code{.cpp}
int t = 0;
while(t < 100) {
    driver.setCyclicVelocity(0.5);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    t++;
}
@endcode

For more information about the mode check CiA 402 documentation.

Parameters:

* vel - target velocity for the CSV mode
* velOffset - commanded offset of the driver
* torOffset - input value for torque feed forward

Returns: True if the setting of the mode was successful

Returns: Error code if setting the mode was unsuccessful
