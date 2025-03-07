@ingroup group_soemapi

[SOEM](https://openethercatsociety.github.io/doc/soem/index.html), or Simple Open EtherCAT Master, is an open-source EtherCAT master library that allows developers to implement EtherCAT communication in their applications. EtherCAT (Ethernet for Control Automation Technology) is a real-time industrial Ethernet communication protocol widely used in automation systems.

### Description

SOEM addresses the fundamental physical layer of EtherCAT communication, serving as the cornerstone for CiA 301, facilitating communication between the master and the slaves. The SOEM library encompasses several key functionalities:

- SDO Messaging
- IOMap Configuration (PDO Mapping)
- EtherCAT State Machine

SDO messaging facilitates asynchronous communication between the master and the slaves, with messages sent asynchronously. Conversely, PDO messaging supports synchronous communication between the master and the slaves, with messages sent in a synchronized manner.

PDO messaging boasts advantages in terms of data throughput over the network. In SDO messaging, sending an SDO object through the network consumes only 4 bytes of memory for sending/receiving a value to/from a register. This memory usage includes the slave's ID and the data size. In PDO messaging, the PDO object requires 8 bytes.

The IOMap represents process data from RPDO and TPDO objects, and its layout is calculated and created by SOEM. The IOMap serves as the payload in an EtherCAT datagram, utilizing logic read/write commands. An EtherCAT frame may contain 1-N datagrams, and the size is determined by the sum of active/selected RPDO/TPDOs. The user must understand the sizes of the IO maps that each slave needs and generate an IO map of at least that size.

The EtherCAT state machine parallels that of the CAN protocol, albeit with different setups and states. The states a slave goes through include init, pre-operational, safe-operational, and operational. Operational mode indicates the slave is ready for use.

### ESI files

For CANopen utilizing the CAN protocol, EDS/DCF files are employed, whereas for CANopen over EtherCAT, ESI files come into play. These files serve as configuration files, manifested as XML documents with a specific element hierarchy. They delineate the physical properties of the slave, along with intricate details about the communication protocol. Similar to EDS/DCF files, these documents encapsulate the object dictionary and include essential information such as the vendor ID, product code, and revision number.

In the context of the SOEM library, the use of ESI files is not mandatory for configuring the master and slaves within the network. However, it's worth noting that having the ESI file can be advantageous, offering users insights into the specifics of the device they are operating.
