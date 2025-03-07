@ingroup device_manager_with_auto_initialization

The DeviceManagerWithAutoInitialization class offers essential functionalities for managing our devices. Its primary feature is the automated handling of device initialization and deinitialization based on request activity and a user-defined timeout. This high-level interface streamlines device management, ensuring efficient resource utilization by keeping the device in a ready state when not actively in use. Additionally, it includes the "get status" function, which needs to be customized by each device to provide essential information about its components.

The timeout parameter is set in the class constructor, allowing flexibility for customization through inheritance.

### How to create your manager

To create your custom manager, you can utilize the DeviceManagerWithAutoInitialization class as a foundation. For now, you can follow the example set by [RPSensor](@ref reference_missing) as a reference implementation.
