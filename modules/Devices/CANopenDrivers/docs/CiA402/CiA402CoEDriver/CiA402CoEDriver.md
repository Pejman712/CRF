@ingroup group_cia_four_zero_two_coe_driver

### Description

The class CiA402CoEDriver presents the class for the drivers that are EtherCAT compliant with the CiA 402 profile of CANopen. The class inherits attributes from the BasicEtherCATDriver class. This class has the implementation of all the possible modes of operation of the CiA 402 profile as well as the implementations of initializing, deinitializing, stopping and reseting a fault in the driver.

With the help of the JSON file (which is one of the attributes that defines an object of this class), the configuration registers of the possible modes can be set. Compared with the CiA402CANDriver class, the PDO mapping for this class is done inside the JSON file. For more information check the user guide.

Among the basic set up and mode of operation functions there are ones that are meant for getting current values of position, velocity, tourqe, statusword and mode of operation.
All of the aforementioned functions are in the public section of the class.

The bindIOMap function binds the rpdos and tpdos that are set up in the IOMap. The function is part of the protected section of the class-

Because of the different time waiting for going to one transition from another in the state machine for the drivers, triggerTransition fucntion and waitTransition are defined. These functions are called every time a slave has to go into another state in the state machine of CiA 402 profile. If there is a timeout, the logger will print that message out, if the driver goes into fault the logger will also print out that message out, otherwise the transition is succcessfull.

Other important functions that are used in the class are setting up the possible modes of operation, writing values in registers with SDOs with writeRegisters function and the change mode of operation function.

All of the aforementioned functions are in the private section of the class.

The set up of possible modes of operation function allows the user to see which modes are available for the driver (which is of high importance). The writing registers function gives a cleaner and more compact SDO writing inside the class.
Changing the modes of operation with the implemented function gives a timeout on transitioning from one mode to another, as well as information if the driver is in fault. If everything is ok, the driver moves to the target mode with this function.

### Implementation of a new motor

When wanting to implement a new motor that is EtherCAT compliant, this is the class that should be used for inheritance. More on how to implement a new motor can be read from the developer guide.
