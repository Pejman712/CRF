@ingroup group_active_tool

This is the documentation for the API of the Active Tool Communication Point. The Active Tool is ******************** available in the CRF.

- [Active Tool Communication Point {#active\_tool\_communication\_point}](#active-tool-communication-point-active_tool_communication_point)
  - [Notes](#notes)
  - [Available API calls](#available-api-calls)

### Notes

This communication point inherits from the Priority Access Communication Point, please refer to [Device Manager](@ref device_manager) to view the common functionalities such as the structure of JSON replies or errors.

For the units refer to [CRF Standard Units](@ref standard_units).

This class uses custom types from the CRF. They might be referred to in JSON as TaskPose, JointPositions, TaskVelocity, etc... To understand how these objects are converted into JSON please refer to [Types JSON Converters](@ref types_json_converters)

Finally, some return types may include the type crf::expected<T>. To get more information on this type and understand how it's parsed in JSON refer to [Error Handler](@ref error_handler)


### Available API calls

The specific API calls will be inside the structure provided by the Priority Access Communication Point and the Status Streamer Communication Point. This is particularly important regarding the general message structure and the methods lockControl and unlockControl.

Particularly, the new instructions will be placed under the message field, following the unified structure of all communication points


#### Activate

Call to activate the robot. Requires PRIORITY.

```json
{
    "command" : "activate",
}
```

#### Deactivate

Call to deactivate the robot. Requires PRIORITY.

```json
{
    "command" : "deactivate",
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
    "isActive" : crf::expected<bool>,
}
```
As an explanation of these fields:
- Status: Indicates if the device has been initialized or deinitialized.
- Status: Indicates the status of the robot. It's an array of error codes.

