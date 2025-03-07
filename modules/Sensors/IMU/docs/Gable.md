@ingroup group_gable

### Introduction
GABLE IMU™ (ETHERCAT® device IMU + GPS + COMPASS) is a company that produces Inertial Measurement Units exclusively designed for EtherCAT® or EtherCAT P® networks. Equipped with premium XSENS® inertial measurement sensors and available in a robust IP67 housing or as a PCB only version for OEM applications.
They sell a broad range of sensor options including GPS, inertial navigation and compass (GNSS/INS RTK).
The website is [GABLE-IMU website](https://gable-imu.nl/#Home). Please check the website for the datasheets.

They currently (January 2024) sell two series of devices;
- ONE SERIES: mounts inside an XSENS®1 MTi-1 series;
- 600 SERIES: mounts inside an XSENS® 2 MTi-6x0.
Gable provide the electronics and the firmware for an EtherCAT communication with the devices.

The ONE-SERIES contains an XSENS®1 MTi-1 series IMU with a 3-axis gyroscope, 3-axis accelerometer and 3-axis magnetometer. Gable integrates the MTi-1 series IMU in a fully functional EtherCAT slave which coordinates the communication between the EtherCAT network and the MTi-1 series IMU. This is realized using a state of art EtherCAT Slave Controller (ESC) and an industrial grade microcontroller (MCU).

The ONE-SERIES is available in several EtherCAT® ready variants:
- SE1: Inertial Measurement Unit (IMU)
- SE2: IMU + Vertical Reference Unit (VRU)
- SE3: IMU + VRU + Altitude and Heading Reference System (AHRS)
- SE5: Global Navigation Satellite System (GNSS)
- SE7: IMU + VRU+ AHRS+GNSS + Inertial Navigation System (INS)

### Tested products
The tested currently products (January 2024) belong to the ONE SERIES. In particular the code has been tested on the SE1, SE2 and SE3 models.
