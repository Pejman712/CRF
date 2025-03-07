@ingroup group_cia_four_zero_two

The profile 402 within the CANopen protocol is primarily associated with the "Device Profile for Drives and Motion Control." This profile outlines communication and control standards for devices related to motion control, such as electric drives, servo drives, and other motion control components. It defines the interaction between master devices (like controllers) and slave devices (like drives) to ensure seamless communication and coordination of motion-related tasks.

CiA profile 402 specifies various objects, parameters, and functions that enable features like speed control, position control, torque control, and other motion-related operations. It establishes a standardized way for devices to exchange information, set control parameters, and monitor motion-related data in a consistent manner, regardless of the manufacturer or specific implementation.

In summary, CiA profile 402 within the CANopen protocol serves as a standardized framework for communication and coordination between motion control devices, facilitating efficient and reliable motion control within industrial and automation applications.

### Modes of operation

One of the key aspects of this profile are the modes of operation. These modes enable devices like drives and controllers to interact in different ways to achieve specific objectives. These modes provide a standardized framework for controlling motion-related tasks and interactions between devices in industrial and automation settings. By selecting the appropriate mode, engineers and operators can achieve precise control over various motion control applications. Some common modes of operation within CiA profile 402 include:

* Profile position mode
* Profile velocity mode
* Profile torque mode
* Velocity mode
* Interpolated position mode
* Homing mode
* Cyclic synchronous position mode
* Cyclic synchronous velocity mode
* Cyclic synchronous torque mode

#### Configuration

In order to configure the behaviour of each mode of operation there is a general configuration file. A template can be found inside the config folder, Template. This JSON file needs to have ONLY the modes of operation available inside the motor, and the registers available for that motor too. Not all motors use the same registers or even all of them so ensure that the registers are correct.

Each register follows the name given by the standard. Each motor can use it's own units for these registers so take them into account. In order to standarize the output to move all the motors in the same units we offer additional configuration parameters:

- PositionUnitConversion: Conversion to transform the position units of the motor into any other units the user desires.
- VelocityUnitConversion: Conversion to transform the velocity units of the motor into any other units the user desires.
- AccelerationUnitConversion: Conversion to transform the acceleration units of the motor into any other units the user desires.
- TorqueUnitConversion: Conversion to transform the torque units of the motor into any other units the user desires.
- GearboxRatio: Ratio of the gearbox if it is not taken into account by the driver of the motor. If there is no gearbox or it is already taken into account by the driver then this parameter can be removed and it will default to 1.

*IMPORTANT* The GearboxRatio is not affecting position, we assume that in most cases the position encoder will be placed after the gearbox and the velocity encoder will be before the gearbox. As such, we only take it into account in velocity to control the final velocity but ensure the driver closes the loop on the pre-gearbox velocity.

All of these unit changes affect in the following way to the units:

$$  finalPosition = measuredPosition / PositionUnitConversion $$
$$  finalVelocity = measuredVelocity / (VelocityUnitConversion * GearboxRatio) $$
$$  finalAcceleration = measuredAcceleration / (AccelerationUnitConversion * GearboxRatio) $$
$$  finalTorque = measuredTorque * GearboxRatio / TorqueUnitConversion $$

This conversion is also done backwards when sending a reference to the motor. This way the user can work in any desired unit even if it's different from the motor one. This allows standarization of motors from a control point of view.
As a final example, when converting to radians you just have to state the correlation between the units of the motor and a full turn.

- From degrees in the motor to radians in the driver -> PositionUnitConversion = 360 / 2*pi
- From milli-degrees in the motor to radians in the driver -> PositionUnitConversion = 360000 / 2*pi
- From 8bit encoder increments in the motor to radians in the driver -> PositionUnitConversion = 2e8 / 2*pi

Basically, how many of the motor units correspond to a full turn.

For a linear motor, like the ERB415Linear one, the units just have to be converted to meters.

- From millimeters in the motor to meters in the driver -> PositionUnitConversion = 1000
