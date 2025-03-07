@ingroup group_cia_four_zero_two_can_driver

### Description

The class CiA402CANDriver presents the class for the drivers that are CAN compliant with the CiA 402 profile of CANopen. The class inherits attributes from the LoopDriver class from the Lely library.
This class has the implementation of all the possible modes of operation of the CiA 402 profile as well as the implementations of initializing,
deinitializing, stopping and reseting a fault in the driver.

With the help of the JSON file (which is one of the attributes that defines an object of this class), the configuration registers of the possible modes can be set. For this class the PDO mapping is done inside the YML file. More information on the YML can be read in the Lely guide.

Among the basic set up and mode of operation functions there are ones that are meant for getting current values of position, velocity, tourqe, statusword and mode of operation.
All of the aforementioned functions are in the public section of the class.

In the private section of the class, the functions that are from the Lely library are overwritten, such as: OnBoot, OnDeconfig and OnEmcy.

OnBoot is called every time the slave is being booted, OnEmcy is called when there is an EMCY error and OnDeconfig is called inside the destructor.

Because of the different time waiting for going to one transition from another in the state machine for the drivers, triggerTransition fucntion and waitTransition are defined. These functions are called every time a slave has to go into another state in the state machine of CiA 402 profile. If there is a timeout, the logger will print that message out, if the driver goes into fault the logger will also print out that message out, otherwise the transition is succcessfull.

Other important functions that are used in the class are setting up the possible modes of operation, writing values in registers with SDOs with writeRegisters function and the change mode of operation function.

The set up of possible modes of operation function allows the user to see which modes are available for the driver (which is of high importance). The writing registers function gives a cleaner and more compact SDO writing inside the class.
Changing the modes of operation with the implemented function gives a timeout on transitioning from one mode to another, as well as information if the driver is in fault. If everything is ok, the driver moves to the target mode with this function.

### Implementation of a new motor

When wanting to implement a new motor that is CAN compliant, this is the class that should be used for inheritance. More on how to implement a new motor can be read from the developer guide.
