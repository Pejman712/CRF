@ingroup group_cia_four_zero_two_communication_point

This is the documentation for the API of the CiA402 Communication Point. The CiA402 is ******************** available in the CRF.

### Notes

This communication point inherits from the Priority Access Communication Point, please refer to [Device Manager](@ref device_manager) to view the common functionalities such as the structure of JSON replies or errors.

For the units refer to [CRF Standard Units](@ref standard_units).

This class uses custom types from the CRF. They might be referred to in JSON as TaskPosition, JointsPosition, TaskVelocity, etc... To understand how these objects are converted into JSON please refer to [Types JSON Converters](@ref group_types_json_converters)

Finally, some return types may include the type crf::expected<T>. To get more information on this type and understand how it's parsed in JSON refer to [Error Handler](@ref group_error_handler)


### Available API calls

The specific API calls will be inside the structure provided by the Priority Access Communication Point and the Status Streamer Communication Point. This is particularly important regarding the general message structure and the methods lockControl and unlockControl.

Particularly, the new instructions will be placed under the message field, following the unified structure of all communication points


#### setProfilePosition

Call to set profile position of the robot. The pos is the target position of the motor, vel is the maximum velocity of the profile type for this mode, acc and dec are acceleration and deceleration of the motor accordingly, refernce is parameter for setting the type of position : absolute or relative and endVel is the velocity the motor has when it reaches the target position (the default value is 0).
Requires PRIORITY.

```json
{
    "command" : "setProfilePosition",
}
```

##### Set Profile Position Message

The data field constitutes a double of position, double velocity, double acceleration, double deceleration and an Absolute position reference. For positionReference a message will be reutrned based off initial request with either "0" for "Absolute" position or "1" for "Relative."

```json
{
    "position" : 0.0,
    "velocity" : 0.0,
    "acceleration" : 0.0,
    "deceleration" : 0.0,
    "positionReference" : 0,
}
```

The return message will be a JSON reply with a message of crf::exepted<bool> type.

#### setProfileVelocity

Call to set profile velocity of the robot. The vel is the target velocity of the motor, vel is the maximum velocity of the profile type for this mode, acc and dec are acceleration and deceleration of the motor accordingly.
Requires PRIORITY.

```json
{
    "command" : "setProfileVelocity",
}
```

##### Set Profile Velocity Message

The data field constitutes a double velocity, double acceleration, double deceleration.

```json
{
    "velocity" : 0.0,
    "acceleration" : 0.0,
    "deceleration" : 0.0,
}
```

The return message will be a JSON reply with a message of crf::exepted<bool> type.

#### setProfileTorque

Function that sets the profile torque mode. The function accepts one parameter which is the target torque. The target torque is the input value for the torque controller in profile torque mode.
Requires PRIORITY.

```json
{
    "command" : "setProfileTorque",
}
```

##### Set Profile Torque Message

The data field constitutes a double torque.

```json
{
    "torque" : 0.0,
}
```

The return message will be a JSON reply with a message of crf::exepted<bool> type.

#### setVelocity

Function that sets the velocity mode. The function accepts five parameters. The vel is desired velocity (or target velocity) of the motor. DeltaSpeedAcc and deltaTimeAcc are used to configure the acceleration of the motor (the acceleration ramp) : acc = deltaSpeedAcc/deltaTimeAcc. DeltaSpeedDec and deltaTimeDec are used to configure the deceleration of the motor (the deceleration ramp) : dec = deltaSpeedDec/deltaTimeDec.
Requires PRIORITY.

```json
{
    "command" : "setVelocity",
}
```

##### Set Velocity Message

The data field constitutes a double velocity, double delta speed acceleration, double delta time accerlation, double delta speed deceleration and double delat time deceleration.

```json
{
    "velocity" : 0.0,
    "deltaSpeedAcc" : 0.0,
    "deltaTimeAcc" : 0.0,
    "deltaSpeedDec" : 0.0,
    "deltaTimeDec" : 0.0,
}
```

#### setMaximumTorque

Function that sets the maximum torque allowed in the slave. The function accepts one parameter which is the slave torque.
Requires PRIORITY.

```json
{
    "command" : "setMaximumTorque",
}
```

##### Set Maximum Torque Message

The data field constitutes a double torque.

```json
{
    "maximumTorque" : 0.0,
}
```

The return message will be a JSON reply with a message of crf::exepted<bool> type.

#### setInterpolatedPosition

Function that sets the interpolated position mode. The function accepts four parameters: pos, vel, acc and dec. The vel is the maximum velocity of the profile type for this mode, acc and dec are acceleration and deceleration of the motor accordingly.
Requires PRIORITY.

```json
{
    "command" : "setInterpolatedPosition",
}
```

##### Set Interpolated Position Message

The data field constitutes a double position, double velocity, double acceleration, double deceleration.

```json
{
    "position" : 0.0,
    "velocity" : 0.0,
    "acceleration" : 0.0,
    "deceleration" : 0.0,
}
```

The return message will be a JSON reply with a message of crf::exepted<bool> type.

#### setModeOfOperation

Function that sets the mode of operation. The function accepts one parameter: mode, which represents the desired mode of operation for the motor. Possible modes of operation can bee seen in the hpp file "CiA402Definitions.hpp".
Requires PRIORITY.

```json
{
    "command" : "setModeOfOperation",
}
```

##### Set Mode Of Operation Message

The data field constitutes an ModeOfOperation object. Return message in mode with a number corresponding to an enum. Please refer to [CiA402Definitions](@ref group_cia_four_zero_two) under 'Classes' and 'ModeOfOperationValues'

```json
{
    "mode" : 0x01,
}
```

The return message will be a JSON reply with a message of enum Type : int8_t.

#### setCyclicPosition

Function that configures the registers for the cyclic synchronous position mode. The function accepts four parameters: pos, posOffset, velOffset and torOffset. The pos is the desired position of the CSP mode (the position that is the reference for the position controler) whilst posOffset is the commanded offset of the driver and the velOffset and torOffset are the input values for velocity and torque feed forward.
Requires PRIORITY.

```json
{
    "command" : "setCyclicPosition",
}
```

##### Set Cyclic Position Message

The data field constitutes a double position, double position offset, double velocity offset, double torque offset.

```json
{
    "position" : 0.0,
    "posOffset" : 0.0,
    "velOffset" : 0.0,
    "torOffset" : 0.0,
}
```

The return message will be a JSON reply with a message of crf::exepted<bool> type.

#### setCyclicVelocity

Function that configures the registers for the cyclic synchronous velocity mode. The function accepts three parameters: vel, velOffset and torOffset. The vel is the velocity of the CSV mode (the velocity that is the reference for the velocity controler) whilst velOffset is commanded offset of the driver and the torOffset is the input value for torque feed forward.
Requires PRIORITY.

```json
{
    "command" : "setCyclicVelocity",
}
```

##### Set Cyclic Velocity Message

The data field constitutes a double velocity, double velocity offset, double torque offset.

```json
{
    "velocity" : 0.0,
    "velOffset" : 0.0,
    "torOffset" : 0.0,
}
```

The return message will be a JSON reply with a message of crf::exepted<bool> type.

#### setCyclicTorque

Function that configures the registers for the cyclic synchronous torque mode. The function accepts three parameters: tor and torOffset. The tor is the desired torque of the CST mode(the torque that is the reference for the torque controler) whilst torOffset is the commanded offset of the driver.
Requires PRIORITY.

```json
{
    "command" : "setCyclicTorque",
}
```

##### Set Cyclic Torque Message

The data field constitutes a double torque, double torque offset.

```json
{
    "torque" : 0.0,
    "torOffset" : 0.0,
}
```

The return message will be a JSON reply with a message of crf::exepted<bool> type.

#### inFault

Function that puts checks if the slave is in fault state.
Requires PRIORITY.

```json
{
    "command" : "inFault",
}
```

#### inQuickStop

Function that puts checks if the slave is in quick stop state.
Requires PRIORITY.

```json
{
    "command" : "inQuickStop",
}
```

#### quickStop

Function that puts the motor in quick stop state. The only time the motor can go to quick stop state is when the motor is in operation enabled state.
Requires PRIORITY.

```json
{
    "command" : "quickStop",
}
```

#### stop

Function that puts the motor into stop (stops the movement of the motor). Modes in which a motor can go into stop are : PPM, PVM, PTM, IPM and VM mode. For cyclic modes this function performs a quickstop.
Requires PRIORITY.

```json
{
    "command" : "stop",
}
```

#### resetFault

Function that resets the motor if it is in fault state. The function checks whether the motor is in fault state, if it is, resets it to switch on disabled state, if not it does nothing.
Requires PRIORITY.

```json
{
    "command" : "resetFault",
}
```

#### resetQuickStop

Function that returns the motor from quickstop state to operation enabled state. This function can be used for any mode when a quickstop is performed.
Requires PRIORITY.

```json
{
    "command" : "resetQuickStop",
}
```

#### Get Status

The "get status" command returns general information of the ActiveTool.

```json
{
    "command" : "getStatus",
}
```

The return message will be a JSON reply with a JSON message containing all the properties of the ActiveTool:

```json
{
    "status" : "initialized / deinitialized",
    "priorityUnderControl" : 1,
    "motorStatus" : std::vector<crf::ResponseCode>,
    "statusWord" : "string",
    "position" : crf::expected<double>,
    "velocity" : crf::expected<double>,
    "torque" : crf::expected<double>,
    "modeOfOperation" : crf::devices::canopendrivers::ModeOfOperation,
    "maxTorque" : crf::expected<double>,
    "inFault" : bool,
    "inQuickstop" : bool,

}
```

For more details on the above and their class definitions please go to [ICiA402Driver](@ref group_cia_four_zero_two). Under 'Classes' and 'ICiA402Driver' for the list of public member functions and how they are defined.

For more details on 'ModeOfOperation' possible modes, please refer to [CiA402Definitions](@ref group_cia_four_zero_two) under 'Classes' and 'ModeOfOperationValues'

As an explanation of these fields:
- Status: Indicates if the device has been initialized or deinitialized.
- Status: Indicates the status of the robot. It's an array of error codes.
