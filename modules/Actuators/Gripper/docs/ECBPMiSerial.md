@ingroup group_ecbpmiserial

The module for Schmalz ECBPMi Vacuum Gripper contains the configuration code for the control of the gripper.

The following commands, derived through reverse engineering, are utilized to control the ECBPMi Vacuum Gripper. They include instructions for suction, blowing-off, standby, activation, and deactivation.

 - **Low Intensity Suction:** [0xF1, 0x01, 0x10 , 0x0A, 0x00, 0X0C, 0xF1]
 - **Medium Intensity Suction:** [0xF1, 0x01, 0x2D, 0x27, 0x00, 0X46, 0xF1]
 - **High Intensity Suction:** [0xF1, 0x01, 0x3C , 0x1E, 0x00, 0X4C, 0xF1]
 - **Blow Off:** [0xF1, 0x02, 0x3C , 0x1E, 0x00, 0X4D, 0xF1]
 - **Stand By:** [0xF1, 0x00, 0x3C , 0x1E, 0x00, 0X4B, 0xF1]
 - **Activate:** [0xF8, 0x78, 0xF8, 0x01, 0x87, 0xE0, 0x00, 0xEA, 0x06, 0x04, 0x20, 0x74, 0xA0, 0x00, 0xFA, 0x00, 0x1A, 0xA0, 0x0E, 0x31, 0x30, 0x2E, 0x30, 0x33, 0x2E, 0x30, 0x31, 0x2E, 0x30, 0x30, 0x35, 0x38, 0x34, 0xDE, 0xA0, 0x00, 0x15, 0x00, 0x35, 0xA0, 0x07, 0x32, 0x37, 0x38, 0x30, 0x30, 0x32, 0x38, 0x12, 0xA0, 0x00, 0xF0, 0x00, 0x10, 0xA0, 0x14, 0x0A, 0x0E, 0x01, 0x01, 0x03, 0x02, 0x02, 0x00, 0x82, 0x00, 0x00, 0x00, 0xEA, 0x01, 0x87, 0xE0, 0x00, 0x2A, 0x6B, 0x7C, 0x3A]
 - **Deactivate:** [0xF1, 0x00, 0x3C , 0x1E, 0x00, 0X4B, 0xF1]

The serial communication has to be stablished with the parity bit enabled
The implementation of ECBPMi derives from crf::devices::tool::IActiveTool. This interface has 5 main functions:

#### Initialize
This function initializes the serial communication, by invoking the "initialize" method of crf::communication::serialcommunication::ISerialCommunication.
In addition to this it sends the unlock command to the ECBPMi, following the right behavior of communication.
After that the pump is ready to receive commands, so the atomic variable *isRunning* is setted at true, meaning that it is in the correct execution path.

#### Deinitialize
This function deinitializes the communication, but before that it ensures that the ECBPMi is stopped and setted to a standBy state in which it is not doing anything, neither releasing air nor sunctioning.
After that it closes the communication by invoking the "denitialize" method of crf::communication::serialcommunication::ISerialCommunication.
In addition to that it also ensure to terminate the execution of threads and sets back the atomic variable *isRunning* to false.

#### Activate
This function activates the pump, meaning that it starts to generate a vacuum to pick up objects with the sunction cups.
This function creates a thread that calls a private function "control" that sends the adeguate command in loop, following the right behavior of communication.
It will stop sending it only when the "deactivate" will be invoked.
The created thread will take two atomic variables for his work. The first is *isRunning*, meaning that if the gripper is not active, it won't call any function.
The second one is *currentOption*, so the option that the user will choose as input. 
If it is == 1, the activate function will be called; if it is == 2, the deactivate function will be called and so the loop sending the activate command will be stopped.

#### Deactivate
This function deactivates the pump, meaning that it will blow-off some air to release the object picked up before.
The function uses for a few milliseconds the thread created by the activated function but changing the *currentOption* at 2. So it sends the proper command to the gripper.
Then it stops the thread and send the command to set the pump in stand by mode.

#### isActive
This function just check if the program is running, it is true when initialized and ready to receive inputs, it is false after deinitializing.
