@ingroup group_canopen_master

A CANopen Master is a central component in a Controller Area Network (CAN) communication system that orchestrates and manages communication with multiple CANopen slave devices. The CANopen Master plays a pivotal role in configuring, controlling, and coordinating the actions of various CANopen slave devices within a network. It initiates communication cycles, handles network synchronization, and facilitates the exchange of process data and configuration information, ensuring seamless interoperability among connected devices.

LELY's AsyncMaster is an implementation of a CANopen Master provided by LELY. The AsyncMaster distinguishes itself by employing an asynchronous communication model, enabling non-blocking and parallel communication with multiple CANopen slave devices. This approach enhances the efficiency of communication by minimizing delays and allowing for concurrent interactions with different slaves. LELY's AsyncMaster is designed to offer flexibility, scalability, and robustness in managing complex CANopen networks. It leverages asynchronous communication patterns to optimize data exchange and responsiveness in diverse industrial settings, making it a valuable solution for applications demanding high-performance CANopen communication.

The CANopenMaster inherits from AsyncMaster to import all of it's features. In addition, the paths to each slaves configuration are dynamically created to be portable among different systems.

**VERY IMPORTANT**
This means that all of the paths in the yaml files for the EDS files **MUST** start from "cpproboticframework". Otherwise the communication won't work.
