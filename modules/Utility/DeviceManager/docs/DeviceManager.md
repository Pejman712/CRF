@ingroup group_device_manager

The device manager plays a vital role in the CRF communications, providing interfaces for various classes that form the foundation of most communication points and clients. These classes are designed to be inherited, allowing for flexibility and customization in the communication structure.

In the context of communication points, there are two main layers available:

- Status Streamer Communication Point
- Priority Access Communication Point

For managers, the device manager offers two layers of functionality:

- Device Manager With Auto Initialization
- Device Manager With Priority Access

Lastly, for clients, two layers are available:

- Status Streamer Client
- Priority Access Client

By inheriting from these classes, developers can build upon the existing functionalities, enabling seamless integration of custom communication points, managers, and clients within the CRF ecosystem.
