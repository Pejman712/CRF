@ingroup device_manager_with_priority_access

The DeviceManagerWithPriorityAccess class is a subclass of the @ref device_manager_with_auto_initialization class, inheriting all of its functionalities. On top of that, this class introduces the management of priority requests and maintains a record of the client currently using the device.

Whenever a client requests access to the device, it gains control, and the control remains locked until the corresponding client explicitly releases it. However, if the control is not renewed within a specific timeout set in the constructor, the control is automatically released, allowing other clients to utilize the device.
