@ingroup group_maxon_epos_two_can_driver

### EDS/DCF file

The EDS file presents the object dictionary of the driver. All of the registers that the driver has, are
in the EDS file. There are manufacturer specific registers, CiA 301 and CiA 402 registers defined in the
file.

For the master, a DCF file needs to be generated for the program to run. Using the Lely library it can be
done in this way:

    dcfgen -r MaxonEPOS2.eds

Using the -r option, remote PDO mapping is configured inside the master.dcf file. Except for PDO
mapping, the specifics of a driver are defined, such as a serial number, vendor id etc. This file
is called when the program is runned for the driver. Explanation is given in the steps for activating
the driver.

For more information on how to use dcfgen you can consult the [Lely CANopen website](https://opensource.lely.com/canopen/docs/dcf-tools/).

### Possible modes of operation

The possible modes of operation for this driver are:

* profile position mode
* profile velocity mode
* interpolated position mode

The MaxonEPOS2 does not have any restrictions regarding accessing certain registers, but there are some that are not present for certain modes. The non present registers are configuring registers that are not of big importance for the modes to work. The most important thing is that the commanding registers are present for the modes.

### Setting up PDOs

As explained in the user guide for lely library (CANopen over CAN protocol), setting up the PDOs is done inside the yml file.
In the yml file, in the master section, the node id of the master needs to defined as well as the sync period of sending PDOs.
For MaxonEPOS2 driver usually 50ms or 100ms can be set for the sync period. In the section for rpdo mapped values, the target position and target velocity need to be defined by their according indexes and subindexes. Depending on what the user wants to read from the driver (statusword, actual position, actual velocity etc), in the tpdo.
section the indexes and subindexes of those registers can be defined.

When wanting to operate with more drivers (for example more wheels of the platform of the CERNbot2), new sections have to be created
for each driver (slave) with their own node id defined, as well as tpdos and rpdos.

For example:

```yml
  master:
    node_id: 5
    sync_period: 50000 # us
    heartbeat_consumer: true
    heartbeat_producer: 50 # ms

  slave_3:
    dcf: "MaxonEPOS2.eds"
    dcf_path: "cpproboticframework/modules/Devices/CANopenDrivers/config/MaxonEPOS2/"
    node_id: 3
    heartbeat_consumer: true
    heartbeat_producer: 50 # ms

    rpdo:
      1:
        enabled: true
        cob_id: auto
        transmission: 0x01
        mapping:
          - {index: 0x607A, sub_index: 0x00} # target position
          - {index: 0x60FF, sub_index: 0x00} # target velocity
      2:
        enabled: true
        cob_id: auto
        transmission: 0x01
        mapping:
        # value with 4 byte
          - {index: 0x60C1,  sub_index: 0x01} # interpolated position

    tpdo:
      1:
        enabled: true
        cob_id: auto
        transmission: 0x01
        mapping:
        #both values have 4 bytes
          - {index: 0x6041, sub_index: 0x00} # statusword
          - {index: 0x6064, sub_index: 0x00} # actual position value register
      2:
        enabled: true
        cob_id: auto
        transmission: 0x01
        mapping:
        # value with 6 bytes
          - {index: 0x6061,  sub_index: 0x00} # modes of operation display
          - {index: 0x606C,  sub_index: 0x00} # velocity actual value
      3:
        enabled: true
        cob_id: auto
        transmission: 0x01
        mapping:
        # value with 4 byte
          - {index: 0x60C1,  sub_index: 0x01} # interpolated position

  slave_4:
    dcf: "MaxonEPOS2.eds"
    dcf_path: "cpproboticframework/modules/Devices/CANopenDrivers/config/MaxonEPOS2/"
    node_id: 4
    heartbeat_consumer: true
    heartbeat_producer: 50 # ms

    rpdo:
      1:
        enabled: true
        cob_id: auto
        transmission: 0x01
        mapping:
          - {index: 0x607A, sub_index: 0x00} # target position
          - {index: 0x60FF, sub_index: 0x00} # target velocity
      2:
        enabled: true
        cob_id: auto
        transmission: 0x01
        mapping:
        # value with 4 byte
          - {index: 0x60C1,  sub_index: 0x01} # interpolated position

    tpdo:
      1:
        enabled: true
        cob_id: auto
        transmission: 0x01
        mapping:
        #both values have 4 bytes
          - {index: 0x6041, sub_index: 0x00} # statusword
          - {index: 0x6064, sub_index: 0x00} # actual position value register
      2:
        enabled: true
        cob_id: auto
        transmission: 0x01
        mapping:
        # value with 6 bytes
          - {index: 0x6061,  sub_index: 0x00} # modes of operation display
          - {index: 0x606C,  sub_index: 0x00} # velocity actual value
      3:
        enabled: true
        cob_id: auto
        transmission: 0x01
        mapping:
        # value with 4 byte
          - {index: 0x60C1,  sub_index: 0x01} # interpolated position
```


IMPORTANT: The user needs to choose if they are going to send values to a register through an SDO or PDO (the register needs to be first PDO mappabale, which can be seen in the EDS file). If values to a register are being sent both through a PDO and SDO, a problem will arrise where certain messages will not be sent to the driver (messages will be skipped). In that case, the user needs to choose whether they want messages to be sent synchronously or asynchronously.

For more information on how to use the yml file you can consult the [Lely CANopen website](https://opensource.lely.com/canopen/docs/dcf-tools/).

### JSON file

Inside the JSON file the configuring registers for the modes are defined as well as the units of the motor.
Because the motor is a rotary motor, all the units are divided by 2*pi. The configuring registers for the motor
define the limits for velocity, acceleration, deceleration, position, sync period, position window etc. For
the limits check the documentation for the values. The JSON file for MaxonEPOS2 driver is MaxonEPOS2.json file.

The JSON file is used for the driver to see which modes are available, as well to set up configuring registers
for the driver. An example JSON file is added under name Template.json.

### Activating the driver

The activation of the driver has to go thorugh these few steps:

* Setting up the can of the driver:

    ./scripts/setCAN.h 0 1000000

If there are more CAN compliant devices in the network, the same line of code is called just with a different id (instead of 0 it can be 1, 2 etc).

* With ip addr check if the can is UP
If it is UP continue, else if it is DOWN and you are unable
to connect, check if the cabling of the CAN device is correct otherwise ask someone who is more skilled in hardware to check it.

* Set up the main program for activating the driver.
You can refer to the MaxonEPOS2Sample file in the sample files.

* Running the program:

    sudo - E ./bin/MaxonEPOS2_sample ../modules/Devices/CANopenDrivers/config/MaxonEPOS2/master.dcf

The user has to run the program with sudo because of access to the can controller. After running this line of the code the user needs to put in their username and password and then the program will run. The - E is added for seeing the logger messages of the program. Before being able to see any of the logger messages this command has to be called in the terminal:

export LOGGER_ENABLE_LOGGING=1 && export LOGGER_PRINT_STDOUT=1 && export LOGGER_ENABLE_DEBUG=1

### Possible errors that can occur

* Value of object 1018 sub-index 01 from CANopen device is different to value in object 1F85 (Vendor-ID)
If this error appears but the values are the same, then you are not connected to the correct can controller.
You need to connect to the right one for this error not to appear.

* Certain SDO abort codes - When an SDO abort code happens check first the connection with the
can controller. If everything is alright with the can controller, try to debug the code and seeing
where the errors occur. The list of the SDO abort codes can be found on [MicroControl website](https://microcontrol-umic.github.io/CANopen-Master-Library/group__SDO__ERR.html).

* Skipping certain messages - This occurs when a register is PDO mapped but it is also being used
for sending/receiving an SDO message. In this case, the user has to choose if they are going to send data
through an SDO or a PDO for the register.

* The motor is not moving in profile velocity mode - It can be that the values that are being sent of the velocity,
acceleration, deceleration are too big or too small for the driver to move the motor. Check the documentation
of the motors that you are trying to move for units and limits.

* The motor is not moving in interpolated position mode - It can be that the new set-point for interpolation
is too small or too big for the movement of the motor. Try to test out different values to see what works best. If that is not the case, then it can be the sync period. Change the value of the sync period, try increasing it, decreasing it, and see if the motor manages to move.

* The motor is not moving in profile position mode - As for the profile velocity mode, check the units and limits of the motor if the velocity, acceleration, deceleration are too big or small.
