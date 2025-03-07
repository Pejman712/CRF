@ingroup group_motion_controller_communication_point

This is the documentation for the API of the Motion Controller Communication Point. The Motion Controller is the main controller class for the movement of the robots available in the CRF.

Keep in mind that even though all the functions are available it does not mean that all robots or controllers can implement them. This might lead to errors in the style of MethodNotAllowed or MethodNotImplemented.

### Notes

This communication point inherits from the Priority Access Communication Point, please refer to [Device Manager](@ref device_manager) to view the common functionalities such as the structure of JSON replies or errors.

For the units refer to [CRF Standard Units](@ref standard_units).

This class uses custom types from the CRF. They might be referred to in JSON as TaskPose, JointPositions, TaskVelocity, etc... To understand how these objects are converted into JSON please refer to [Types JSON Converters](@ref types_json_converters)

Finally, some return types may include the type crf::expected<T>. To get more information on this type and understand how it's parsed in JSON refer to [Error Handler](@ref error_handler)

### Available API calls

The specific API calls will be inside the structure provided by the Priority Access Communication Point and the Status Streamer Communication Point. This is particularly important regarding the general message structure and the methods lockControl and unlockControl.

Particularly, the new instructions will be placed under the message field, following the unified structure of all communication points


#### Append Path

Function to add a path for the robot to follow. If the robot is already following a path then the new one will be added to the end of the queue.

The call can have two possible structures, depending if the path sent is in task space or joint space.

```json
{
    "command" : "appendPath",
}
```

**IMPORTANT**: Consecutive paths sent with the same types (e.g. appending a task space path when a task space path is already running) will add the new points to the end of the queue. However, changing types while a path is running (e.g. appending a task space path when a joint space path is running) will result in the interruption of the current path and an immediate change into the new path.

##### Append Joints Path Message

The data field constitutes a vector of joint positions. Every joint position needs to have the same dimensions and they need to be coherent with the robot's dimensions, otherwise, an error will be returned.

```json
{
    "type" : "joints",
    "data" : [[0, 0, 0, 0, 0, 0], ...]
}
```

The return message will be a JSON reply with a message of crf::exepted<bool> type.

##### Append Task Path Message

The data field constitutes a vector of Task positions. An example is given in the code, for more information on how to write a task position in JSON please refer to LINK TO TASK POSITION

Methods:
- 1 - Joint space trajectory
- 2 - Task space trajectory

Reference:
- 1 - Global reference frame
- 2 - TCP reference frame


```json
{
    "type" : "task",
    "data" : [{"data" : [0, 0, 0, 0, 0, 0], "representation" : 1}, {...}, ...],
    "method" : 1,
    "reference" : 1
}
```

The return message will be a JSON reply with a message of crf::exepted<bool> type.

#### Set Velocity

Call to ask the robot to follow a certain velocity in task space or joint space. Only one velocity can be sent. If a new velocity is received that will be the new reference and the previous one will be discarded.

```json
{
    "command" : "setVelocity",
}
```

##### Set Joints Velocity Message
To send a joint velocity command, we can follow the next message structure.

```json
{
    "type" : "joints",
    "data" : JointVelocities,
}
```

Where the data needs to have the same amount of elements as the dimensions of the robot.

The return message will be a JSON reply with a message of crf::exepted<bool> type.

##### Set Task Velocity Message

To send a task velocity command we need a new field, reference. This field will represent the reference frame of the task space velocity we send. It follows the next enumeration.

Reference:
- 1 - Global reference frame
- 2 - TCP reference frame

The final message looks like this:

```json
{
    "type" : "task",
    "data" : TaskVelocity,
    "reference" : 1
}
```

The return message will be a JSON reply with a message of crf::exepted<bool> type.

#### Set Torque

Call to ask the robot to produce a torque in task space or joint space. Only one torque can be sent. If a new torque is received that will be the new reference and the previous one will be discarded.

```json
{
    "command" : "setTorque",
}
```

##### Set Joints Torque Message
To send a joint torque command, we can follow the next message structure.

```json
{
    "type" : "joints",
    "data" : JointForceTorques,
}
```

Where the data needs to have the same amount of elements as the dimensions of the robot.

The return message will be a JSON reply with a message of crf::exepted<bool> type.


##### Set Task Torque Message

To send a task torque command we need a new field, reference. This field will represent the reference frame of the task space velocity we send. It follows the next enumeration.

Reference:
- 1 - Global reference frame
- 2 - TCP reference frame

The final message looks like this:

```json
{
    "type" : "task",
    "data" : TaskForceTorque,
    "reference" : 1
}
```

Where the data needs to have the same amount of elements as the task space dimensions of the robot.

The return message will be a JSON reply with a message of crf::exepted<bool> type.

#### Set Profile Velocity

Sets the cruise velocity when following a path. This velocity can be set for joint paths and task paths respectively in joint space or task space.

```json
{
    "command" : "setProfileVelocity",
}
```

##### Set Joints Profile Velocity Message
To send a joint torque command, we can follow the next message structure.

```json
{
    "type" : "joints",
    "data" : JointVelocities,
}
```

Where the data needs to have the same amount of elements as the joints of the robot.

The return message will be a JSON reply with a message of crf::exepted<bool> type.


##### Set Task Profile Velocity Message

```json
{
    "type" : "task",
    "data" : TaskVelocity,
}
```

Where the data needs to have the same amount of elements as the task space dimensions of the robot.

The return message will be a JSON reply with a message of crf::exepted<bool> type.

#### Set Profile Acceleration

Sets the cruise acceleration when following a path. This acceleration can be set for joint paths and task paths respectively in joint space or task space.

```json
{
    "command" : "setProfileAcceleration",
}
```

##### Set Joints Profile Acceleration Message
To send a joint acceleration command, we can follow the next message structure.

```json
{
    "type" : "joints",
    "data" : JointAccelerations,
}
```

Where the data needs to have the same amount of elements as the joints of the robot.

The return message will be a JSON reply with a message of crf::exepted<bool> type.

##### Set Task Profile Acceleration Message

```json
{
    "type" : "task",
    "data" : TaskAcceleration,
}
```

Where the data needs to have the same amount of elements as the task space dimensions of the robot.

The return message will be a JSON reply with a message of crf::exepted<bool> type.

#### Soft stop

The soft stop command will trigger a controlled stop. It will stop while following the current path and not deviating.

All the current paths will be deleted. The client will have to resend the next points to follow. A soft stop can be triggered by the controller if certain circumstances are met (e.g. robot going into singularity, out of bounds, ...). These circumstances will depend on the controller used.

The soft stop acts within the deceleration limits stated in the controller and does not stress the hardware in any way. As such, it should be the default way of stopping the robot in non-critical situations.

```json
{
    "command" : "softStop",
}
```

The return message will be a JSON reply with a message of crf::exepted<bool> type.

#### Hard Stop

A hard stop will trigger an immediate stop. This might result in brake activation and might damage the robot in the long term. As such, this stop should only be used in critical circumstances where a full stop is crucial.

```json
{
    "command" : "hardStop",
}
```

The return message will be a JSON reply with a message of crf::exepted<bool> type.

#### Set Parameters

This function acts as a way to interact with the controller. This function can activate special functions related to the controller, the inverse kinematics, ... One noticeable example is the Inverse Kinematics Objective Functions which can be triggered using this method. These functions have to be set up beforehand and might not be available in all circumstances.

```json
{
    "command" : "setParameters",
}
```

The return message will be a JSON reply with a message of crf::exepted<bool> type.

#### Get Status

The "get status" command returns general information from the motion controller.

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
    "isTrajectoryRunning" : true,
    "controllerStatus" : [401, 2003, ...],
    "jointPositions" : JointPositions,
    "jointVelocities" : JointVelocities,
    "jointAccelerations" : JointAccelerations,
    "jointForceTorques" : JointForceTorques,
    "taskPose" : TaskPose,
    "taskVelocity" : TaskVelocity,
    "taskAcceleration" : TaskAcceleration,
    "taskForceTorque" : TaskForceTorque
}
```
As an explanation of these fields:
- Status: Indicates if the controller has been initialized or deinitialized
- IsTrajectoryRunning: Indicates if a trajectory is currently being executed
- Status: Indicates the status of the controller and robot. It's an array of error codes.
- Joint and Task values: Indicate the current signals of the robotic system.

#### Get Configuration

The "get configuration" command returns general and static information of the robot.

```json
{
    "command" : "getConfiguration",
}
```

The return message will be a JSON reply with a JSON message containing all the properties of the controller:

```json
{
}
```