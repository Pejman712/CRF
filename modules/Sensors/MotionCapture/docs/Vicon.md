@ingroup group_vicon

### Introduction
[Vicon](https://www.vicon.com/) is the leading developer of motion capture products and services for the life sciences, virtual reality, entertainment and engineering industries.

The tested products are _Vicon Vero v2.2_ cameras, controlled via _Vicon Tracker_ software (version 3.10).

### How to set up the configuration file
The options contained in the configuration are divided in two categories: System setup (ConnectionTimeout, ClientBufferSize, StreamingType, AxisMapping) and DataStreaming (Lightweight and ObjectsOnly).

#### ConnectionTimeout
Value consisting in the connection timeout in millisecond, default is 5000 milliseconds, minimum is 10
milliseconds. The connection tentative will fail if no connection was able to be made within Timeout milliseconds.

#### ClientBufferSize
Value consisting in the maximum number of frames the client should buffer. The default value is 1, which always supplies the latest frame. Choose higher values to reduce the risk of missing frames between calls.

#### StreamingType
There are three modes that the SDK can operate in. Each mode has a different impact on the Client, Server, and network resources used.
- "ServerPush" mode = 0
In "ServerPush" mode the Server pushes every new frame of data over the network to the Client. The Server will try not to drop any frames. This results in the lowest latency that can be achieved. If the Client is unable to read data at the rate it is being sent, then it is buffered, firstly in the Client, then on the TCP/IP connection, and then at the Server. When all the buffers are full then frames may be dropped at the Server and the performance of the Server may be affected. 
- "ClientPull" mode = 1
In "ClientPull" mode, the Client requests the latest frame of data from the Server. This increases latency, because a request must be sent over the network to the Server, the Server has to prepare the frame of data for the Client, and then the data must be sent back over the network. Network bandwidth is kept to a minimum, because the Server only sends what you need. The buffers are very unlikely to be filled, and Server performance is unlikely to be affected. 
- "ClientPullPreFetch" mode = 2
"ClientPullPreFetch" is an enhancement to the "ClientPull" mode. As soon as a sample has been received, it will preemptively request the next sample. The server will send you this next sample as soon as it is ready, so do not
experience the delay in requesting it.  As with normal "ClientPull", buffers are unlikely to fill up, and Server performance is unlikely to be affected. 

The stream defaults to "ClientPull" mode as this is considered the easiest option. For improved performance use "ServerPush". "ClientPullPreFetch" may be useful if problems are being caused by large numbers of samples being buffered.

#### AxisMapping
Vicon Data uses a right-handed coordinate system, with +X forward, +Y left, and +Z up. Other systems use different coordinate systems. The SDK can transform its data into any valid right-handed coordinate system by re-mapping each axis. Valid directions are "Up", "Down", "Left", "Right", "Forward", and "Backward". Note that "Forward" means moving away from you, and "Backward" is moving towards you. Common usages are:
- "Z-up" mapping = 0
Corresponds to the following mapping: X-Forward, Y-Left, Z-Up
- "X-up" mapping = 1
Corresponds to the following mapping: X-Forward, Y-Up, Z-Right
- "Y-up" mapping = 2
Corresponds to the following mapping: X-Up, Y-Forward, Z-Left

#### Lightweight
Enables a lightweight transmission protocol for kinematic segment data in the Vicon DataStream. This will reduce the network bandwidth required to transmit segment data to approximately a quarter of that required by the previous method, at the expense of a small amount of precision.

#### ObjectsOnly
Enables/disables the streaming of markers data.

### How to launch the sample
For launching the sample one needs to:
1. Connect and power the Vicon cameras
2. Place markers on the desired object to track
3. Access to the machine on which the Vicon software is installed (e.g. Vicon Tracker), open the software and turn on the tracking of the object
4. Retrieve the Host Name of the machine (e.g. pcbe17326) or its public IP address
5. Connect to a PC on the CERN network and run the following commands

````bash
cd build
make ViconSample
./../bin/ViconSample --host_name pcbe17326 --configuration ../modules/Sensors/MotionCapture/config/ViconConfig.json
````
