@ingroup group_elmo_gold

### Possible modes of operation

The possible modes of operation for this driver are:

* profile position mode
* profile velocity mode
* profile torque mode
* cyclic synchronous position mode
* cyclic synchronous velocity mode
* cyclic synchronous torque mode

The ELMOGoldSoloTwitter does not have any restrictions regarding  accessing certain registers, but there are some that are not present for certain modes. The non present registers are configuring registers that are not of big importance for the modes to work. The most important thing is that the commanding registers are present for the modes.

Unlike the CAN compliant drivers, the EtherCAT drivers are more precise with real-time data. In that case the sync period (cycle time) can be just 1ms for sending real-time data. EtherCAT compliant drivers have a nanosecond accuracy when processing real-time data.

### Setting up PDOs

Setting up the PDOs for this driver is done inside the JSON file. In the section PDO of the JSON file the PDO mapped registers can be deifned.

For example:

    "PDO" : {
        "TXPDO" : [
            {"Idx" : "1A0A", "SubIdx" : 1},
            {"Idx" : "1A0B", "SubIdx" : 2},
            {"Idx" : "1A0E", "SubIdx" : 3},
            {"Idx" : "1A11", "SubIdx" : 4},
            {"Idx" : "1A13", "SubIdx" : 5},
            {"Idx" : "1A1F", "SubIdx" : 6}
        ],
        "RXPDO" : [
            {"Idx" : "160A", "SubIdx" : 1},
            {"Idx" : "160B", "SubIdx" : 2},
            {"Idx" : "160F", "SubIdx" : 3},
            {"Idx" : "161C", "SubIdx" : 4},
            {"Idx" : "160C", "SubIdx" : 5},
            {"Idx" : "161A", "SubIdx" : 6}
        ]
    }

The idx presents the index of the PDO that is being mapped. These indexes can define individual registers for the motor, or they can define a group of registers that are being PDO mapped. Check in the documentation of the motor which registers you need for PDO mapping and then write the according PDO mapping indexes in the JSON file.

IMPORTANT: The user needs to choose if they are going to send values to a register through an SDO or PDO (the register needs to be first PDO mappabale, which can be seen in the documentation of the driver). If values to a register are being sent both through a PDO and SDO, a problem will arrise where certain messages will not be sent to the driver (messages will be skipped). In that case, the user needs to choose whether they want messages to be sent synchronously or asynchronously.

### JSON file

Inside the JSON file the configuring registers for the modes are defined as well as the units of the motor. 
Because the motor is a rotary motor, all the units are divided by 2*pi. The configuring registers for the motor
define the limits for velocity, acceleration, deceleration, position, sync period, position window etc. For 
the limits check the documentation for the values. The JSON file for ELMOGoldSoloTwitter driver is ELMOGoldSoloTwitter.json file.

The JSON file is used for the driver to see which modes are available, as well to set up configuring registers
for the driver. An example JSON file is added under name Template.json.

### Activating the driver

The activation of the driver has to go thorugh these few steps:

* Checking the name of the port of the driver with ip addr
* Putting that as ifname in the main program
* Set up the main program for activating the driver.
You can refer to the ELMOGoldTwitterSample file in the sample files.
* Running the program:

    sudo - E ./bin/ELMOGoldTwitterSample

The user has to run the program with sudo because of access to the can controller. After running this line of the code the user needs to put in their username and password and then the program will run. The - E is added for seeing the logger messages of the program. Before being able to see any of the logger messages this command has to be called in the terminal:

export LOGGER_ENABLE_LOGGING=1 && export LOGGER_PRINT_STDOUT=1 && export LOGGER_ENABLE_DEBUG=1

### Possible errors that can occur

* Certain SDO abort codes - When an SDO abort code happens check first the connection with the 
can controller. If everything is alright with the can controller, try to debug the code and seeing 
where the errors occur. The list of the SDO abort codes can be found on [MicroControl website](https://microcontrol-umic.github.io/CANopen-Master-Library/group__SDO__ERR.html).

* Skipping certain messages - This occurs when a register is PDO mapped but it is also being used
for sending/receiving an SDO message. In this case, the user has to choose if they are going to send data
through an SDO or a PDO for the register.

* The motor is not moving in profile velocity mode - It can be that the values that are being sent of the velocity,
acceleration, deceleration are too big or too small for the driver to move the motor. Check the documentation
of the motors that you are trying to move for units and limits.

* The motor is not moving in one of the cyclic modes and interpolated position mode - It can be that the new set-point for the trajectory is too small or too big for the movement of the motor. Try to test out different values to see what works best. If that is not the case, then it can be the sync period. Change the value of the sync period, try increasing it, decreasing it, and see if the motor manages to move.

* The motor is not moving in profile position mode - As for the profile velocity mode, check the units and limits of the motor if the velocity, acceleration, deceleration are too big or small.

* The motor is not moving in profile torque mode - As for the profile velocity mode, check the units and limits of the motor if the torque units are too big or small.


Motor rated torque (MRT): 10000 mNm -> 10 Nm  -> 1000

500 -> 5000mNm

500 / 1000 = 0.5 -> 0.5 * MRT = 5000 -> 5000 * 14 = 70000 -> 70000 / 1000 = 70 Nm

xsend / 7.14
xread * 7.14

Motor Rated Current
