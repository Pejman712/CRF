#pragma once
/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <KinovaTypes.h>
#include <Kinova.API.CommLayerUbuntu.h>

namespace crf::actuators::kinovaarm {

class IKinovaApiInterface {
 public:
    virtual ~IKinovaApiInterface() = default;
    /*
     * Initializes the API. It is the first function you call if you want the rest of the
     * methods
     * Returns NO_ERROR_KINOVA = 1 : if operation is a success.
     * Returns ERROR_LOAD_COMM_DLL = 2002 : if unable to load the communication layer.
     * Returns ERROR_INIT_COMM_METHOD = 2006 : if unable to load the InitComm function from 
     * communication layer.
     * Returns ERROR_CLOSE_METHOD = 2007 : if unable to load the Close function from 
     * communication layer.
     * Returns ERROR_GET_DEVICE_COUNT_METHOD = 2008 : if unable to load the GetDeviceCount
     * function from communication layer.
     * Returns ERROR_SEND_PACKET_METHOD = 2009 : if unable to load the SendPacket function
     * from communication layer.
     * Returns ERROR_SET_ACTIVE_DEVICE_METHOD = 2010 : if unable to load the SetActiveDevice
     * function from communication layer.
     * Returns ERROR_GET_DEVICES_LIST_METHOD = 2011 : if unable to load the GetDevices function
     * from communication layer.
     * Returns ERROR_SCAN_FOR_NEW_DEVICE = 2013 : if unable to load the ScanForNewDevice 
     * function from the communication layer.
     * Returns ERROR_GET_ACTIVE_DEVICE_METHOD = 2014 : if unable to load the GetActiveDevice
     * function from the communciation layer.
     * Returns ERROR_OPEN_RS485_ACTIVATE = 2015 : if unable to load the OpenRS485_Activate
     * function from the communication layer.
     */
    virtual int ethernetInitEthernetAPI(EthernetCommConfig& config) = 0; // NOLINT
    /*
     * Must called when your application stops using the API. It closes the USB link and
     * the library properly.
     * Returns NO_ERROR_KINOVA = 1 - Always returns success.
     */
    virtual int ethernetCloseAPI() = 0;
    /*
     * Once recieved by the robot, this method tells the robotical arm that the API will 
     * control it from this point forward. It must be called before sending trajectories 
     * or any commands via the joystick.
     * Returns NO_ERROR_KINOVA = 1 : if operation is a success.
     */
    virtual int ethernetStartControlAPI() = 0;
    /*
     * Once recieved by the robot, this method tells the robotical arm that the API will 
     * not control it anymore from this point forward.
     * Returns NO_ERROR_KINOVA = 1 : if operation is a success.
     */
    virtual int ethernetStopControlAPI() = 0;
    /*
     * This method refresh the devices list connected on the USB bus.
     * Returns NO_ERROR_KINOVA = 1 : if operation is a success.
     * Returns -1 : Input/output error.
     * Returns -2 : Invalid parameter.
     * Returns -3 : Access denied (insufficient permissions)
     * Returns -4 : No such device (it may have been disconnected)
     * Returns -5 : Entity not found.
     * Returns -6 : Resource busy.
     * Returns -7 : Operation timed out.
     * Returns -8 : Overflow.
     * Returns -9 : Pipe error.
     * Returns -10 : System call interrupted (perhaps due to signal)
     * Returns -11 : Insufficient memory.
     * Returns -12 : Operation not supported or unimplemented on this platform.
     * Returns -99 : Other error.
     */
    virtual int ethernetRefresDevicesList() = 0;
    /*
     * This method returns a list of devices accessible by th API.
     * The result of the operation is in the parameter "result" [out] and it can be
     *  NO_ERROR_KINOVA = 1 : If operation is a success
     *  ERROR_NO_DEVICE_FOUND = 1015 : If no kinova device is found on the bus
     *  ERROR_API_NOT_INITIALIZED = 2101 : If the function ethernetInitEthernetAPI(_)
     * has not been called previously.
     * Returns the devices count found on the USB bus.
     */
    virtual int ethernetGetDevices(KinovaDevice devices[MAX_KINOVA_DEVICE], int& result) = 0; // NOLINT
    /*
     * This function sets the current active device. The active device is the device that 
     * will receive the command send by this API. If no active device is set, the first 
     * one discovered is the default active device.
     * Returns NO_ERROR_KINOVA = 1 : if operation is a success.
     * Returns ERROR_API_NOT_INITIALIZED = 2101 : if the API has not been initialized
     */    
    virtual int ethernetSetActiveDevice(KinovaDevice device) = 0;
    /*
     * This moves the robot to the HOME position, also known as the READY position.
     * Returns NO_ERROR_KINOVA = 1 : if operation is a success.
     */
    virtual int ethernetMoveHome() = 0;
    /*
     * This function initializes the fingers of the robotical arm. After the initialization,
     * the robotical arm will be in angular control mode. If you want to use the task 
     * control mode, use the method ethernetSetTaskControl().
     * Returns NO_ERROR_KINOVA = 1 : if operation is a success.
     */
    virtual int ethernetInitFingers() = 0;
    /*
     * This method erases all the trajectories inside the robotical arm's FIFO.
     * Returns NO_ERROR_KINOVA = 1 : if operation is a success.
     */
    virtual int ethernetEraseAllTrajectories() = 0;
    /*
     * This method activates/deactivates the automatic singularity avoidance algorithms.
     * When state=1, protection is activated. When state=0, protection is deactivated. 
     * Returns NO_ERROR_KINOVA = 1 : if operation is a success.
     */
    virtual int ethernetActivateSingularityAutomaticAvoidance(int state) = 0;
    /*
     * This method activates/deactivates the automatic collision avoidance algorithms.
     * When state=1, protection is activated. When state=0, protection is deactivated.  
     * Returns NO_ERROR_KINOVA = 1 : if operation is a success.
     */
    virtual int ethernetActivateCollisionAutomaticAvoidance(int state) = 0;
    /*
     * This function returns the angular position of the robotical arm's end effector.
     * Units are in degrees.
     * The result of the operation is in the parameter "response" [out] and it returns 
     * a structure that contains the position of each actuator and finger.
     * Returns NO_ERROR_KINOVA = 1 : if operation is a success.
     */
    virtual int ethernetGetAngularPosition(AngularPosition& response) = 0; // NOLINT
    /*
     * This function gets the velocity of each actuator. Units are in degrees/second.
     * The result of the operation is in the parameter "response" [out] and it returns 
     * a structure that contains the velocity of each actuator and finger.
     * Returns NO_ERROR_KINOVA = 1 : if operation is a success.
     */
    virtual int ethernetGetAngularVelocity(AngularPosition& response) = 0; // NOLINT
    /*
     * This function gets the torque of each actuator. Units are in Nm.
     * The result of the operation is in the parameter "response" [out] and it returns 
     * a structure that contains the torque of each actuator.
     * Returns NO_ERROR_KINOVA = 1 : if operation is a success.
     */
    virtual int ethernetGetAngularForce(AngularPosition& response) = 0; // NOLINT
    /*
     * This function gets the torque of each actuator gravity free. Units are in Nm.
     * The result of the operation is in the parameter "response" [out] and it returns
     * a structure that contains the torque of each actuator.
     * Returns NO_ERROR_KINOVA = 1 : if operation is a success.
     */
    virtual int ethernetGetAngularForceGravityFree(AngularPosition& response) = 0; // NOLINT
    /*
     * This function returns the task position of the robotical arm's end effector. 
     * The orientation is defined by Euler angles (convention XYZ).
     * The result of the operation is in the parameter "response" [out] and it returns 
     * a structure that contains  the position vector at the end effector.
     * Returns NO_ERROR_KINOVA = 1 : if operation is a success.
     */
    virtual int ethernetGetTaskPose(CartesianPosition& response) = 0; // NOLINT
    /*
     * This function returns the values of the support sensors available in the arm
     * The result of the operation is in the parameter "response" [out] and it returns 
     * a structure that contains various data
     * Returns NO_ERROR_KINOVA = 1 : if operation is a success.
     */
    virtual int ethernetGetTaskForce(CartesianPosition& response) = 0; // NOLINT
    /*
     * This function returns the values of the support sensors available in the arm
     * The result of the operation is in the parameter "response" [out] and it returns 
     * a structure that contains various data
     * Returns NO_ERROR_KINOVA = 1 : if operation is a success.
     */
    virtual int ethernetGetSensorInfo(SensorsInfo& response) = 0; // NOLINT
    /*
     * This function sends a trajectory point (WITH limitation) that will be added in 
     * the robotical arm's FIFO.
     * The parameter trajectory has the trajectory point that you want to send to the
     * robotic arm.
     * Returns NO_ERROR_KINOVA = 1 : if operation is a success.
     */
    virtual int ethernetSendAdvanceTrajectory(TrajectoryPoint trajectory) = 0;
    /*
     * This function sets the robotical arm in task control mode. The robot will 
     * not switch back to Task mode if it is not in a valid Task pose, i.e.
     * too close from a singularity, too close from the base for self-collision avoidance,
     * too close from a joint limit and inside a protection zone. The reason why the
     * robot is always switching back to Task control when you bring it to the Home
     * position is because the Home position is chosen as a valid Task position.
     * Returns NO_ERROR_KINOVA = 1 : if operation is a success.
     */
    virtual int ethernetSetTaskControl() = 0;
    /*
     * This function sets the robotical arm in angular control mode.
     * Returns NO_ERROR_KINOVA = 1 : if operation is a success.
     */
    virtual int ethernetSetAngularControl() = 0;
    /*
     * This function gets the actual control type.
     * The result of the operation is in the parameter "response" [out] and it returns 
     * 0 for Task control type and 1 for Angular control type.
     * Returns NO_ERROR_KINOVA = 1 : if operation is a success.
     */
    virtual int ethernetGetControlType(int& response) = 0; // NOLINT
    /*
    * Sets the current torque measurement as the new zero baseline
    * Returns NO_ERROR_KINOVA = 1 : if operation is a success.
    */
    virtual int ethernetSetTorqueZero(int ActuatorAdress) = 0; // NOLINT
    /*
     * This function starts the admittance control of the arm.
     * The result of the operation is in the parameter "response" [out] and it returns
     * Returns NO_ERROR_KINOVA = 1 : if operation is a success.
     */
    virtual int startForceControl() = 0;
    /*
    * This function stops the admittance control of the arm.
    * The result of the operation is in the parameter "response" [out] and it returns
    * Returns NO_ERROR_KINOVA = 1 : if operation is a success.
    */
    virtual int stopForceControl() = 0;
    /*
    * Sets the damping and inertia parameters for angular admittance control mode.
    * The result of the operation is in the parameter "response" [out] and it returns
    * Returns NO_ERROR_KINOVA = 1 : if operation is a success.
    */
    virtual int setAngularInertiaDamping(AngularInfo inertia, AngularInfo damping) = 0;
    /*
    * Sets min and max torques of the angular admittance control mode.
    * The minimum torque is the minimum for the robot to start moving
    * The max force is the torque that the joint can exert at maximum
    * The result of the operation is in the parameter "response" [out] and it returns
    * Returns NO_ERROR_KINOVA = 1 : if operation is a success.
    */
    virtual int setAngularTorqueMinMax(AngularInfo min, AngularInfo max) = 0;
    /*
    * Sets the damping and inertia parameters for task admittance control mode.
    * The result of the operation is in the parameter "response" [out] and it returns
    * Returns NO_ERROR_KINOVA = 1 : if operation is a success.
    */
    virtual int setTaskInertiaDamping(CartesianInfo inertia, CartesianInfo damping) = 0;
    /*
    * Sets min and max torques of the task admittance control mode.
    * The minimum torque is the minimum for the robot to start moving
    * The max force is the torque that the joint can exert at maximum
    * The result of the operation is in the parameter "response" [out] and it returns
    * Returns NO_ERROR_KINOVA = 1 : if operation is a success.
    */
    virtual int setTaskForceMinMax(CartesianInfo min, CartesianInfo max) = 0;
};

}  // namespace crf::actuators::kinovaarm
