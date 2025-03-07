@ingroup group_linear_actuator_communication_point

This is the documentation for the API of the Linear Actuator Communication Point. The Linear Actuator is the updated class for the Linear Stage available in the CRF.

- Notes
- Available API calls
    - Set Position
        - Set Position Message
    - Set Velocity
        - Set Velocity Message
    - Get Status

### Notes

This communication point inherits from the Priority Access Communication Point, please refer to [Device Manager](@ref device_manager) to view the common functionalities such as the structure of JSON replies or errors.

For the units refer to [CRF Standard Units](@ref standard_units).

This class uses custom types from the CRF. They might be referred to in JSON as TaskPose, JointPositions, TaskVelocity, etc... To understand how these objects are converted into JSON please refer to [Types JSON Converters](@ref types_json_converters)

Finally, some return types may include the type crf::expected<T>. To get more information on this type and understand how it's parsed in JSON refer to [Error Handler](@ref error_handler)

### Available API calls

The specific API calls will be inside the structure provided by the Priority Access Communication Point and the Status Streamer Communication Point. This is particularly important regarding the general message structure and the methods lockControl and unlockControl.

Particularly, the new instructions will be placed under the message field, following the unified structure of all communication points

#### Set Position

Call to ask the robot actuator to be set to a certain position. Only one position can be sent. If a new position is received that will be the new reference and the previous one will be discarded.

```json
{
    "command" : "setPosition",
    "position" : 0.0,
    "priority" : 1
}
```

The return message will be a JSON reply with a message of crf::exepted<bool> type.


#### Set Velocity

Call to ask the robot actuator to follow a certain velocity. Only one velocity can be sent. If a new velocity is received that will be the new reference and the previous one will be discarded.

```json
{
    "command" : "setVelocity",
    "velocity" : 0.0,  // around 5e5 for a normal speed, max in gui 1e6
    "priority" : 1
}
```

The return message will be a JSON reply with a message of crf::exepted<bool> type.

#### Get Status

The "get status" command returns general information from the controller.

```json
{
    "command" : "getStatus",
}
```

The return message will be a JSON reply with a JSON message containing all the properties of the controller:

```json
{
    "status" : "initialized / deinitialized",
    "priorityUnderControl" : 1,
    "position" : crf::expected<double>,
    "velocity" : crf::expected<double>,
}
```
As an explanation of these fields:
- Status: Indicates if the device has been initialized or deinitialized.
- Status: Indicates the status of the robot. It's an array of error codes.
