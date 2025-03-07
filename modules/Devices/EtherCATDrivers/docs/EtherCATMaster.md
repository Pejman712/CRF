@ingroup group_ethercat_master

### Description

The EtherCATMaster class presents the class for defining an EtherCAT master. The class uses the SOEMAPI interface (wrapper around the SOEM library) for basic functions like reading/writing SDOs, checking slave communication, sending/receiving process data etc. The master also has the role to check if certain slaves are lost, off or if all of them are in operational mode with the workingCounterCheck() function.

The master inside the EtherCAT network has the duty to set up all the slaves in the network to operational state.
The steps that the slave has to go through when going to operational mode are:

* At the start the slave is in the Init state. No mailbox or process data communication is possible.

* During the transition between Init and Pre-Op the EtherCAT slave checks whether the mailbox was initialized correctly.
In Pre-Op state mailbox communication is possible, but not process data communication.

* During transition between Pre-Op and Safe-Op the EtherCAT slave checks whether the sync manager channels for process data communication and, if required, the distributed clocks settings are correct.
In Safe-Op state mailbox and process data communication is possible, although the slave keeps its outputs in a safe state, while the input data are updated cyclically.

* Before the EtherCAT master switches the EtherCAT slave from Safe-Op to Op it must transfer valid output data.
In the Op state the slave copies the output data of the masters to its outputs. Process data and mailbox communication is possible.

For more information on the state machine check the [Beckhoff website](https://infosys.beckhoff.com/english.php?content=../content/1033/ax5000_usermanual/html/Bt_EcBasics_EcStateMachine.htm&id)

Our EtherCAT master depends on the \ref group_soemapi module, so refer to that for further information.
