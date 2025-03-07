@ingroup group_can_open_drivers

CANopen is a communication protocol designed for industrial automation and control systems. It is based on the Controller Area Network (CAN) bus, which is known for its reliability and robustness. CANopen adds a higher-level application layer to the CAN bus, enabling devices from different manufacturers to communicate and cooperate effectively within various industrial and automation scenarios.

Key characteristics of the CANopen protocol include:

- Standardization: CANopen defines a standardized set of communication rules, object dictionary structures, and device profiles. This standardization ensures interoperability between devices from different vendors.

- Scalability: CANopen supports a wide range of devices, from simple sensors to complex controllers. Its modular design allows for easy integration of new devices into existing networks.

- Flexibility: The protocol provides flexibility in configuring device parameters, modes of operation, and communication methods. This adaptability is crucial for accommodating diverse applications and industries.

- Real-time Communication: CANopen supports real-time data exchange, making it suitable for time-critical applications like motion control, robotics, and manufacturing processes.

- Network Management: CANopen includes network management functions that allow devices to start, stop, and synchronize their operations. This facilitates coordinated actions across the network.

- Device Profiles: Device profiles define specific communication and behavior standards for different types of devices, ensuring consistent operation regardless of the manufacturer.

For more information in any of the following pages you can always refer to the [CANopen Standard](https://www.can-cia.org/). All the information regarding the protocol, profiles, and everything else, can be found inside (beware that some information might require some kind of payment, most notably the access to the standard protocol itself).

### Important information

The main point is to understand the [LELY CANopen Lirbary](https://opensource.lely.com/canopen/) and [SOEM Library](https://openethercatsociety.github.io/doc/soem/). The LELY CANopen library covers the physical communication of CAN and the CiA 301 communication profile of CANopen. The SOEM Library covers the physical communication of EtherCAT. The CiA 301 communication profile is built inside of the CANopenDevices library using SOEM. Additionaly, the CANopenDevices library extends the libraries with the implementation of different CiA profiles.
The libraries will return in some cases and crf::expected container, to understand how this container is used please refer to it's proper documentation.

The tables for understanding the structure of the CANopenDevices library are given underneath. Table 1 presents the structure with CAN protocol and Lely library, whilst table 2 presents the structure with EtherCAT protocol and SOEM library.

### Table 1 - Structure of the CANopenDevices library with CAN protocol and Lely library

|   Layers           | Protocol  |       layers of the library               |
|:----------:        |:--------: | :------------------------------:          |
|Application layer   | CANopen   |CANopenDevices library                     |
|^                   |^          |CiA device profiles 401,402,406...         |
|^                   |^          |Object Dictionary (stored in EDS/DCF files)|
|^                   |^          |CiA 301 - Lely library                     |
|Physical layer      | CAN       | Lely library                              |

### Table 2 - Structure of the CANopenDevices library with EtherCAT protocol and SOEM library

|   Layers          | Protocol  |       layers of the library               |
|:----------:       |:--------: | :------------------------------:          |
|Application layer  | CANopen   |CANopenDevices library                     |
|^                  |^          |CiA device profiles 401,402,406...         |
|^                  |^          |Object Dictionary (stored in EDS/DCF files)|
|^                  |^          |CiA 301                                    |
|Physical layer     | EtherCAT  | SOEM library                              |


### Available Profiles

Profiles in CANopen refer to predefined sets of communication and behavior standards that define how specific types of devices should interact and operate within a CANopen network. These profiles provide a common framework that ensures interoperability between devices from different manufacturers, as they all adhere to the same communication rules and data structures. Some of the known profiles are CiA 401 for I/O modules and the CiA 402 for motion control.

### Setting up a new motor

When setting up the motor the user has to go through these steps:

- Checking the units, available modes and registers from the documentation+
- Writing the JSON file for the configuration of the motor
- In case of CANopen over CAN: writing the YML file for PDO mapping
- In case of CANopen over EtherCAT: Writing PDO mapping inside the JSON file.

When working with a new motor it is important to have supported documentation. The documentation can provide insight into the object dictionary (which registers the motor has), modes of operation and the units. The units are the most important part of the set up of the motor. In the JSON file the units can be defined for the motor. For every motor in the JSON file the user has to set up position units, velocity units, acceleration units and torque units. For more information you can check the CiA402Profile guide.
An example of how it should look like:

```json
    "PositionUnitConversion" : 9126.58,
    "VelocityUnitConversion" : 9126.58,
    "AccelerationUnitConversion" : 9126.58,
    "TorqueUnitConversion": 9126.58,
```

You can also refer to the Template.JSON from the config folder for more information.

The next part in the JSON file is the configuration of the possible modes of operation of the motor. For every mode the configuring registers are defined inside the JSON file.
An example of hwo it should look like:

```json
"ModesOfOperation" : {
        "ProfilePositionMode" : {
            "MaxProfileVelocity" : 50,
            "QuickStopDeceleration" : 100,
            "MaxAcceleration" : 100,
            "MotionProfileType" : "Trapezoidal"
        },
        "ProfileVelocityMode" : {
            "MaxProfileVelocity" : 50,
            "QuickStopDeceleration" : 100,
            "MaxAcceleration" : 100,
            "MotionProfileType" : "Trapezoidal"
        },
        "ProfileTorqueMode" : {
            "MotorRatedTorque" : 70,
            "PositiveTorqueLimitValue" : 100,
            "NegativeTorqueLimitValue" : 100
        }
}
```

You can also refer to the Template.JSON from the config folder for more information.

Check the documentation of the motor and follow which modes are available and which registers are available for the specific mode.

When using a motor that is CAN compliant, a YML file will need to be used for the set up of the PDOs. Using the YML file is thourugly explained in the Lely guide. For the EtherCAT compliant motor, PDOs need to be set up in the JSON file as:

```json
    "PDO" : {
        "TxPDO" : [
            {"Idx" : "1A0A", "SubIdx" : 1},
            {"Idx" : "1A0B", "SubIdx" : 2},
            {"Idx" : "1A0E", "SubIdx" : 3},
            {"Idx" : "1A11", "SubIdx" : 4},
            {"Idx" : "1A13", "SubIdx" : 5},
            {"Idx" : "1A1F", "SubIdx" : 6}
        ],
        "RxPDO" : [
            {"Idx" : "160A", "SubIdx" : 1},
            {"Idx" : "160B", "SubIdx" : 2},
            {"Idx" : "160F", "SubIdx" : 3},
            {"Idx" : "161C", "SubIdx" : 4},
            {"Idx" : "160C", "SubIdx" : 5},
            {"Idx" : "161A", "SubIdx" : 6}
        ]
    }
```

The idx presents the index of the PDO that is being mapped. These indexes can define individual registers for the motor, or they can define a group of registers that are being PDO mapped. Check in the documentation of the motor which registers you need for PDO mapping and then write the according PDO mapping indexes in the JSON file.

### Implementing a new driver

The CiA402CANDriver and CiA402CoEDriver are both classes that give the general functionalities of the CiA402 profile. Every mode for the profile is implemented, as well as the general communication part of CANopen. The only difference is that one has CAN as the basic physcial layer for communication, whilst the other one has EtherCAT.
If the user has a new driver that does not follow the CiA402 standard in entierty, a new class can be derived from these ones for the driver (depending if the driver is CAN or EtherCAT compliant). As an example, in the library there is a dervied class for the ERB415 driver. The ERB415 driver does not have certain registers for the modes that it has and it has restrictions accessing certain registers. In that case a new library had to be created.

### Changing functions

As said before, the user has the ability from both of the classes to change the functions as he wants by overriding them.
As an example from ERB415CANDriver, the user changes the used parameters according to the values used by the driver since some registers are non-existant

Of course, the user has the freedom to alter the functions as he wishes and add more as they wish.
