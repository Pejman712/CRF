@ingroup group_csp

### Mode of operation - CSP

With this mode, the trajectory generator is located in the control device, not in the drive device.
In cyclic synchronous manner, it provides a target position to the drive device, which performs position control, velocity control and torque control (a cascade control system). The drive device can provide measurements (values) for actual position, velocity and torque to the control device. For more information check out the CiA 402 standard.

This mode is different from the PPM and IPM mode since it doesn't use the controlword for activating the movement of the motor. For this mode the parameters need to be sent synchronously via the PDOs. Position, velocity and torque offset are optional.

### Implementation of CSP mode

Function setCyclicPosition(double pos, double posOffset,double velOffset, double torOffset) from ICiA402Driver, configures the registers for the cyclic synchronous position mode.
The function accepts four parameters:

* pos
* posOffset 
* velOffset
* torOffset 

The pos is the desired position of the CSP mode (the position that is the reference for the position controller) whilst posOffset is the commanded offset of the driver and the velOffset and torOffset are the input values for velocity and torque feed forward.

Inside the function, the mode is configured through these steps:

1. checking if the motor is initialized
2. if the motor is initialized the mode of operation is set and the configuring registers are set, else the function returns an error code.
3. the target registers are set (commanding parameters)
4. position paramter value is sent to the slave via PDO

If the function passes through all the parts it returns true, else returns an error code.
The user has the freedom to use this function as he wants in the main. If in any case the input value of the position is over the limit, the motor will go into quickstop. Be aware that when using this mode of operation, the starting position of this mode should be the current position of the motor. 
Note that when sending values to the slave (in this case the motor) they need to be sent synchronously, that is why the values of the position should be sent through a loop.

For example:

@code{.cpp}
crf::expected<double> l = driver.getPosition();
if (!l) return;
double i = l.value();
int t = 0;
while(t < 100) {
    driver.setCyclicPosition(i);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    i += 0.05;
    t++;
}
@endcode

For more information about the mode check CiA 402 documentation.

Parameters:

* pos - target positon for the CSP mode
* posOffset - commanded offset of the driver
* velOffset - input value for velocity feed foward
* torOffset - input value for torque feed forward

Returns: True if the setting of the mode was successful

Returns: Error code if setting the mode was unsuccessful
