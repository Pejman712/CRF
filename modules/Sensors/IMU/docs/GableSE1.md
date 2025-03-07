@ingroup group_gable_se1

### Introduction
The SE1 is an IMU that outputs calibrated 3D rate of turn, 3D acceleration and 3D magnetic field. The SE1 also outputs coning and sculling compensated orientation increments and velocity increments ( Δq and Δv). Advantages over a simple gyroscope-accelerometer combo-sensor are the inclusion of synchronized magnetic field, on-board signal processing and the easy-to-use synchronization and communication protocol. The signal processing pipeline and the suite of output options allow access to the highest possible accuracy at any output data rate up to 800 Hz. Moreover, the testing and calibration is already performed by XSENS® and results in a robust and reliable sensor module, which enables a short time to market for the users.

### Implemented and tested functionalities
The implemented and tested functionalities for Gable SE1 are:
- Reading of Acceleration, Gyroscope, MagneticField. By default data are streamed at 100Hz.
- Reading of device infos and settings (Baudrate, Firmware, Hardware, DeviceID). The filter profile in this IMU model is not available, is then identically zero.

Notes regarding other data streaming:
- All the measures are referred to a local frame. The orientation of this frame can be checked in on the XSENS MTi-1 series documentation.

### Functionalities not yet implemented
The following consideration refers to Gable SE1.
- High Rate Acceleration and Gyroscope. The default configuration of the ONE-SERIES does not include the GyroscopeHR and AccelerationHR PDOs. This is because the time synchronized and processed data at 100Hz is most commonly used. These PDOs can be activated separately by sending the correct command. Activating this option it is possible to stream acceleration and gyroscope data up to 1000Hz.

### Other information
- In the config folder there is the ESI file of Gable SE1 which was requested directly to the company since it cannot be found online.
