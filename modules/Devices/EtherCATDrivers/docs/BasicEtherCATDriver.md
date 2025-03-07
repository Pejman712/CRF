@ingroup group_basic_ethercat_driver

### Description

The BasicEtherCATDriver class serves as the cornerstone for defining an EtherCAT driver (slave), providing a streamlined set of functions encompassing initialization, deinitialization, retrieval of the driver's ID, verification of the driver's status, and binding the IOMap.

Importantly, this class forms the basis for the CiA402CoEDriver class.

In a manner akin to the CAN protocol process, the EtherCAT driver progresses through specific states to achieve operational mode. The sequential steps for the driver (slave) to attain operational mode are as follows:

- Initially, the slave resides in the Init state, where neither mailbox nor process data communication is feasible.
- During the transition from Init to Pre-Op, the EtherCAT slave validates the proper initialization of the mailbox. In the Pre-Op state, mailbox communication becomes possible, though process data communication remains unavailable.
- In the transition from Pre-Op to Safe-Op, the EtherCAT slave ensures the correctness of sync manager channels for process data communication and, if necessary, validates distributed clock settings. In the Safe-Op state, both mailbox and process data communication are possible, with the slave maintaining outputs in a safe state while cyclically updating input data.
- Before the EtherCAT master shifts the EtherCAT slave from Safe-Op to Op, it must transfer valid output data. In the Op state, the slave copies the master's output data to its own outputs, facilitating both process data and mailbox communication.

For additional insights into the state machine, refer to the [Beckhoff website](https://infosys.beckhoff.com/english.php?content=../content/1033/ax5000_usermanual/html/Bt_EcBasics_EcStateMachine.htm&id)
