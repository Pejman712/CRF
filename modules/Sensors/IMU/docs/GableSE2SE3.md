@ingroup group_gable_se2se3

### Introduction
Gable SE2 is a 3D VRU. On top of the functionality of the SE1 (which is an IMU that outputs calibrated 3D rate of turn, 3D acceleration and 3D magnetic field), SE2 algorithm computes 3D orientation data with respect to a gravity referenced frame: drift-free roll, pitch and unreferenced yaw. The raw sensor signals are combined and processed at a high frequency to produce a real-time data stream with device’s 3D orientation (roll, pitch and yaw) up to 100 Hz.

Gable SE3 supports all features of the SE1 and SE2, and in addition is a full magnetometer-enhanced AHRS. In addition to the roll and pitch, it outputs a true magnetic North referenced yaw (heading) and calibrated sensors data: 3D acceleration, 3D rate of turn, 3D orientation and velocity increments (Δq and Δv) and 3D earthmagnetic field data. The raw sensor signals are combined and processed at a high frequency to produce a realtime data stream with device’s 3D orientation (roll, pitch and yaw) up to 400 Hz.

### Implemented and tested functionalities
The implemented and tested functionalities for Gable SE2 and SE3 are:
- Reading of Acceleration, Gyroscope, MagneticField, Quaternions and Euler Angles. 
- Reading of device infos and settings (Baudrate, Firmware, Hardware, FilterProfile, DeviceID).
- Enable and disable Active Heading stabilization (AHS).
- Change filter for the orientation estimation.

Notes regarding data streaming:
- All the measures are referred to a local frame. The orientation of this frame can be checked in on the XSENS MTi-1 series documentation.
- The quaternion is streamed as a structure containg {qw, qx, qy, qz}.
- The euler angles representation is ZYX and the values are in degrees.
- The orientation is referred to a local frame with X and Y axes parallel to the ground.
- By default data are streamed at 100Hz.

Notes regarding yaw data:
- For Gable SE2 although the yaw is unreferenced, the estimate it is superior to only gyroscope integration as a result of advanced on-board sensor fusion, but it can be still subjected to a low drift. The drift in unreferenced heading can be limited by using the Active Heading Stabilization (AHS) functionality.
- For Gable SE3 the yaw is by default North-referenced. The behaviour of the yaw data streaming can be changed by changing filter of by activating AHS. 

### Functionalities not yet implemented
- The 3D acceleration is also available as so-called free acceleration, which has the local-gravity subtracted. The activation of this functionality has not been implemented yet.
- The default configuration of the ONE-SERIES does not include the GyroscopeHR and AccelerationHR PDOs enables. This is because the time synchronized and processed data at 100Hz is most commonly used. These PDOs can be activated separately by sending the correct command. Activating this option it is possible to stream acceleration and gyroscope data up to 1000Hz.

### Active Heading Stabilization (AHS)
Active Heading Stabilization (AHS) is a software component within the sensor fusion engine designed to give a low-drift unreferenced (not North-referenced) yaw solution even in a disturbed magnetic environment. It is aimed to tackle magnetic distortions that do not move with the sensor, i.e. temporary or spatial distortions. The magnetic norm can be used to identify these magnetic distortions. 

AHS is designed to be used with Xsens' VRU/AHT products. When AHS is applied to a filter profile that uses the magnetic field as a reference, the magnetic field will no longer be used as a reference. The yaw output will be referenced with respect to startup heading orientation, instead of North-referenced. Upon initialization of the sensor, the yaw estimate will therefore be 0 degrees. With AHS enabled, the drift in yaw can be as low as 1-3 degrees per hour. This however depends on the type of application. AHS works best for applications that are occasionally motionless, such as warehouse robotics and other ground vehicles. 

When the application expects that either the MTi or the magnetic field rotates very slowly, it is recommended to not enable AHS.

### Filter profiles
SE2 and SE3 can be used with several filter settings depending on the environment and application. The available filters are:
- General (Available for Gable ONE-SERIES SE2 and SE3). Standard filter profile with conservative tuning. Ten seconds resistance to magnetic field changes, then slowly converges to the new magnetic field. Can be used for most applications.
- High magnetometer dependence (Available for Gable ONE-SERIES SE3). Filter profile relies heavily on magnetic field. Around 10 seconds resistance to magnetic field changes, then converges to new magnetic field. Assumes a homogeneous magnetic field. To use in applications that can use the magnetometer and can be magnetic field mapped.
- Dynamic (Available for Gable ONE-SERIES SE3). Filter profile assumes fast changes in magnetic field, but also periods where it doesn’t change. Around 10 seconds resistance to magnetic field changes; then quickly converges to new magnetic field. To use in applications that experience accelerations and dynamic movements, e.g. handheld applications.
- North referenced (Available for Gable ONE-SERIES SE3). Filter profiles assumes a homogeneous magnetic field with short magnetic changes allowed. When there is a long-lasting magnetic disturbance, the heading will very slowly converge to the new magnetic field. To be used in applications that have low magnetic distortions and where fast heading changes are not expected, e.g. Satellite on the Move, buoys.
- VRU general (Available for Gable ONE-SERIES SE2 and SE3). Behavior as in General for roll and pitch (inclination). The heading is not referenced by the magnetic field. The gyro bias however is estimated continuously, even in in the z-axis. Magnetic distortions may have an effect on the gyro bias estimation accuracy. This filter profile is designed to work most effectively with the Active Heading Stabilization (AHS) feature enabled. To be used in applications where the magnetic field cannot be trusted, e.g. ground robotics in industrial environments.

### Other information
In the config folder there is the ESI file of Gable SE2 and SE3 which was requested directly to the company since it cannot be found online. For the function implementing the possibility to modify the configuration of the devices (i.e setting filter profiles or enabling/disabling AHS), the following manuals have been consulted:
- Gable IMU ONE Series Datasheet (Section Configuration and Settings) available at [Gable documentation](https://gable-imu.nl/ONE-SERIES_Datasheet.pdf)
- MT Low Level docuementation available at [MT documentation](https://www.xsens.com/hubfs/Downloads/Manuals/MT_Low-Level_Documentation.pdf)
Please refer to these manuals for clarification
