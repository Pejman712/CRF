@ingroup group_lely_library

### Description

The Lely library represents the foundation of the CiA 301 profile for the communication between the master and the slave.
This library covers the physical as well as part of the application layer of the CANopen over CAN protocol.
This library covers:

* SDO messaging
* PDO messaging
* SYNC messaging
* Error handling
* NMT state machine

SDO messaging presents asynchronous messaging between the master and the slaves. (messages are sent asynchronously)
PDO messaging presents synchronous messaging between the master and the slaves. (messages are sent synchronously)

PDO messaging is better in terms of how much data can be sent through the network. For the SDO messaging, when the SDO object
is being sent through the network it takes only 4 bytes of memory for sending/receiving a value to/from a register (the rest of the memory holds the id of the slave and the size of the data), whilst the PDO object in the PDO messaging takes 8 bytes.

SYNC protocol provides the basic network synchronization mechanism.(sending messages periodically) This is covered by Lely with PDO mapping inside the yml file which will be explained later.

Error handling of the Lely library is covered in a way if an error occurs from the CiA standard, a very precise message will appear in the terminal of the program that the user is running with the explanation of the error. For example, when an SDO error occurs from the [table](https://microcontrol-umic.github.io/CANopen-Master-Library/group__SDO__ERR.html) the error handling part of Lely will explain the error and say from where it comes from. This allows the user to easily find out the part where the error is coming from.

The NMT state machine presents the communication behaviour of a CANopen device. Lely already covers the NMT state machine transitions for the slave when the program is called. The slave goes through initialize, pre-operational and then operational mode. When it is in operational mode it is ready for use.

### YML file

The YML file that Lely is using is a file that contains the PDO mapping of the registers for the network. It can be divided into sections: master, slave_1, slave_2, slave_3 etc. Each of this sections has attributes that are assigned to them by the user.
The master section is defined by the node id, the sync period and having the option of being the heartbeat consumer or heartbeat producer.
The sync period depends on the device that is being controlled by the master. Heartbeat producer/consumer can be activated for the slave/master to see if there is a connection established between them. With the candump the user can check if there is a response message from both the slave and the master when the heartbeat is set up.

The example of a master section is given underneath:

```yml
master:
  node_id: 5
  sync_period: 50000 # us
  heartbeat_consumer: true
  heartbeat_producer: 50 # ms
```

Regarding the slave sections, they all have the same form, they just hold a different id and depending on the device, tpdos and rpdos.
For every slave section, the EDS/DCF file has to be defined, as well as the path to it and the node id. As for the master the heartbeat consumer/producder is set when the user wants to check the connection between the two. The sections of the rpdos and tpdos is where the registers that are PDO mapped are defined through indexes and subindexes.

For example:

```yml
slave_3:
  dcf: "MaxonEPOS4.eds"
  dcf_path: "cpproboticframework/modules/Devices/CANopenDrivers/config/MaxonEPOS4/"
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
        - {index: 0x6071,  sub_index: 0x00} # target torque
  tpdo:
    1:
      enabled: true
      cob_id: auto
      transmission: 0x01
      mapping:
        - {index: 0x6041, sub_index: 0x00} # statusword
        - {index: 0x6064, sub_index: 0x00} # actual position value register
    2:
      enabled: true
      cob_id: auto
      transmission: 0x01
      mapping:
        - {index: 0x6061,  sub_index: 0x00} # modes of operation display
        - {index: 0x606C,  sub_index: 0x00} # velocity actual value
    3:
      enabled: true
      cob_id: auto
      transmission: 0x01
      mapping:
        - {index: 0x6077,  sub_index: 0x00} # torque actual value
```

Since the Lely library uses the EDS/DCF file of a device for PDO mapping, the user has to have it for their application of running a driver. The EDS/DCF file of a slave can be found either online (ERB415) or getting it directly from a software that was built for the specific dirver (MaxonEPOS or ELMO). The DCF file is important not only from the slave side, but also from the masters side.
The EDS file of a slave presents the object dictionary. It contains all the registers that the slave has and its specifications like vendor id, product code, revision number etc. The DCF file of the master contains the information of the PDO mappings and the information on all the slaves that are in the network. The DCF file of the master can be generated as:

```sh
dcfgen -r MaxonEPOS4.eds
```

With this the DCF file of a master is created with the remote PDO mappings. The master.dcf file is later called in the terminal when the program is set to run, and the motor is put in motion:

 sudo - E ./bin/ERB415_sample ../modules/Devices/CANopenDrivers/config/MaxonEPOS4/master.dcf

For more information on how to use the yml file you can consult the [Lely CANopen website](https://opensource.lely.com/canopen/docs/dcf-tools/).

### candump

The candump allows the user to see what is happening inside the network. A candump can be called like:

```sh
    candump can0
```

Then the user will have the opportunity to see something like this:

can0  602   [8]  23 16 10 01 D0 07 01 00
can0  582   [8]  60 16 10 01 00 00 00 00
can0  602   [8]  2B 17 10 00 E8 03 00 00
can0  582   [8]  60 17 10 00 00 00 00 00

This shows two SDO requests for object 1016 and for object 1017. The first configures the heartbeat consumer (object 1016 sub-index 1) of the slave, the second the heartbeat producer (object 1017). The first and thrid line represent what is being sent, whilst the second and fourth line represent the confirmation of the SDO request being received.

For more information on how candump can be read and configured virtually check out the [Lely website](https://opensource.lely.com/canopen/docs/cpp-tutorial/).

### C++ tutorial of Lely

For anyone starting with Lely I do reccomend reading and going through the [tutorial](https://opensource.lely.com/canopen/docs/cpp-tutorial/). It gives you a step through step guide to get to know the library, PDOs, SDOs, master-slave communication etc.
Everything is well explained and there are helpful files to go through in this tutorial.
