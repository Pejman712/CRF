@ingroup group_priority_access_communication_point

The PriorityAccessCommunicationPoint class is a straightforward class that inherits all functionalities from [StatusStreamerCommunicationPoint](@ref status_streamer_communication_point). It serves as an extension of this communication point, implementing additional commands required for device control management.

This class should be used in conjunction with [DeviceManagerWithPriorityAccess](@ref status_streamer_communication_point).

### Lock Control

The "lockControl" function requests control of the device.

```json
{
    "command" : "lockControl",
    "priority" : 1,
}
```

The "priority" field indicates the priority of the client, with 1 being the highest. Each client must use a unique priority.

The server will respond with a JSON reply containing a boolean message. If the response is true, access has been granted to the client. However, if the response is false, it means that another client with a higher priority is currently in control of the device.

### Unlock Control

The "unlockControl" function releases the control of the device, allowing other clients to take control.

```json
{
    "command" : "lockControl",
    "priority" : 1,
}
```

The server will respond with a JSON reply containing a boolean message. If the response is true, access has been successfully released.

If a client disconnects or does not use the priority-restricted functions, control will be automatically released after a specified timeout.
