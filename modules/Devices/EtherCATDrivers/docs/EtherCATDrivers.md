@ingroup group_ethercat_drivers

### Introduction

EtherCAT, or Ethernet for Control Automation Technology, is a high-performance, real-time industrial communication protocol designed for automation applications. Developed by Beckhoff Automation, EtherCAT has gained widespread adoption in the field of industrial automation due to its speed, flexibility, and efficiency.

This module introduces two main classes: An EtherCATMaster, and a BasicEtherCATDriver.

An EtherCAT Master plays a crucial role in an EtherCAT network, serving as the central control unit responsible for coordinating communication with EtherCAT slave devices. Here's an introduction to the key aspects of an EtherCAT Master:

An EtherCAT driver is a software or firmware component responsible for enabling communication between a host system (such as a PC or controller) and an EtherCAT network. The driver facilitates the exchange of data between the host and EtherCAT slave devices. The driver present here is supposed to serve as the basis for more advanced drivers that delve into more advanced concepts. This might include CoE (CANopen Over EtherCAT), IMU drivers, I/O modules, ...
