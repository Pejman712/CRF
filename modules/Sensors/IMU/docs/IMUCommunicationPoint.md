@ingroup group_imu_communication_point

This is the documentation for the communication point of all IMU's. The IMU is an Inertial Measurement Unit (IMU) sensor module that combines multiple inertial sensors to provide accurate and real-time information about an object's movement in three-dimensional space which is available in the CRF.

### Notes

This communication point inherits from the Priority Access Communication Point, please refer to [Device Manager](@ref device_manager) to view the common functionalities such as the structure of JSON replies or errors.

For the units refer to [CRF Standard Units](@ref standard_units).

This class uses custom types from the CRF. They might be referred to in JSON as TaskPosition, JointsPosition, TaskVelocity, etc... To understand how these objects are converted into JSON please refer to [Types JSON Converters](@ref types_json_converters)

Finally, some return types may include the type crf::expected<T>. To get more information on this type and understand how it's parsed in JSON refer to [Error Handler](@ref error_handler)


### Available API calls

The specific API calls will be inside the structure provided by the Priority Access Communication Point and the Status Streamer Communication Point. This is particularly important regarding the general message structure and the methods lockControl and unlockControl.

Particularly, the new instructions will be placed under the message field, following the unified structure of all communication points


#### Calibrate

Call to calibrate the IMU. Requires PRIORITY.

```json
{
    "command" : "calibrate",
}
```


#### Get Status

The "get status" command returns general information of the IMU.

```json
{
    "command" : "getStatus",
}
```

The return message will be a JSON reply with a JSON message containing all the properties of the IMU:

```json
{
    "status" : "initialized / deinitialized",
    "priorityUnderControl" : 1,
    "signals" : {
        "position" : crf::expected<std::array<double3>>,
        "quaternion" : crf::expected<std::array<double4>>,
        "eulerZYX" : crf::expected<std::array<double3>>,
        "linearVelocity" : crf::expected<std::array<double3>>,
        "angularVelocity" : crf::expected<std::array<double3>>,
        "linearAcceleration" : crf::expected<std::array<double3>>,
        "angularAcceleration" : crf::expected<std::array<double3>>,
        "magneticField" : crf::expected<std::array<double3>>
    }
}
```
As an explanation of these fields:
- Status: Indicates if the device has been initialized or deinitialized.
- Status: Indicates the status of the IMU. It's an array of error codes.
- IsTrajectoryRunning: Indicates if a trajectory is currently being executed