/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Hannes Gamper CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>
#include <vector>

namespace crf::communication::universalrobotrtde {

class IUniversalRobotRTDEInterface {
 public:
    virtual ~IUniversalRobotRTDEInterface() = default;

    // Interfaces for rtdeReceive

    /**
     * @brief Establishes the receive connection with the robot
     * @param IP IP address
     * @return void
     */
    virtual void initRtdeReceiveInterface(std::string IP) = 0;

    /**
     * @brief Requests the current joint positions of the robot
     * @return The joints position
     */
    virtual std::vector<double> getActualQ() = 0;

    /**
     * @brief Requests the current joint velocities of the robot
     * @return The joints velocities
     */
    virtual std::vector<double> getActualQd() = 0;

    /**
     * @brief Requests the current task position of the robot
     * @return Actual Cartesian coordinates of the tool: (x,y,z,rx,ry,rz),
     *         where rx, ry and rz is a rotation vector representation of the tool orientation
     */
    virtual std::vector<double> getActualTCPPose() = 0;

    /**
     * @brief Requests the current task velocities of the robot
     * @return The task velocities
     */
    virtual std::vector<double> getActualTCPSpeed() = 0;

    /**
     * @brief Requests the current task forces of the robot
     * @return The task forces
     */
    virtual std::vector<double> getActualTCPForce() = 0;

    /**
     * @brief Requests the current robot status
     * @return The robot status bits 0-3:
     *         Is power on | Is program running | Is teach button pressed | Is power button pressed
     */
    virtual uint32_t getRobotStatus() = 0;

    /**
     * @brief Requests the current robot mode
     * @return The robot mode:
     *         -1 = ROBOT_MODE_NO_CONTROLLER, 0 = ROBOT_MODE_DISCONNECTED,
     *         1 = ROBOT_MODE_CONFIRM_SAFETY, 2 = ROBOT_MODE_BOOTING, 3 = ROBOT_MODE_POWER_OFF,
     *         4 = ROBOT_MODE_POWER_ON, 5 = ROBOT_MODE_IDLE, 6 = ROBOT_MODE_BACKDRIVE,
     *         7 = ROBOT_MODE_RUNNING, 8 = ROBOT_MODE_UPDATING_FIRMWARE
     */
    virtual int32_t getRobotMode() = 0;

    /**
     * @brief Requests the current robot safety status bits
     * @return The robot safety status bits 0-10:
     *         Is normal mode | Is reduced mode | Is protective stopped | Is recovery mode |
     *         Is safeguard stopped | Is system emergency stopped | Is robot emergency stopped |
     *         Is emergency stopped | Is violation | Is fault | Is stopped due to safety
     */
    virtual uint32_t getSafetyStatusBits() = 0;

    // Interfaces for rtdeControl

    /**
     * @brief Establishes the control connection with the robot
     * @param IP IP address
     * @return void
     */
    virtual void initRtdeControlInterface(std::string IP) = 0;

    /**
     * @brief Direct servo control of robot to desired position. Smooth trajectory necessary.
     * @param qDes [rad] Desired joint positions
     * @param maxVel [rad/s] Not used in current version
     * @param maxAcc [rad/s^2] Not used in current version
     * @param loopTime [s] The function is blocking for control loop time
     * @param lookAheadTime [s] The robot is waiting for lookAheadTime before sending the desired
     *        joint positions to the robot. Range = [0.03, 0.2].
     * @param gain [1] Proportional gain to follow the desired joint positions. Range = [100, 2000].
     * @return True in case of success and False in case of problems.
     */
    virtual bool servoJ(std::vector<double> qDes, double maxVel, double maxAcc, double loopTime,
        double lookAheadTime, double gain) = 0;

    /**
     * @brief Direct servo control of the desired joint velocities. Continues with constant
     *        velocity. Smooth trajectory necessary.
     * @param qdDes [rad/s] Desired joint velocities
     * @param maxAcceleration [rad/s^2] Acceleration of the leading axis
     * @param loopTime [s] The function is blocking for control loop time (optional)
     * @return True in case of success and False in case of problems.
     */
    virtual bool speedJ(std::vector<double> qdDes, double maxAcceleration, double loopTime) = 0;

    /**
     * @brief Direct servo control of the desired task velocities. Continues with constant
     *        velocity. Smooth trajectory necessary.
     * @param zdDes [m/s] and [rad/s] Desired task velocities
     * @param maxAcceleration [m/s^2] Task position acceleration
     * @param loopTime [s] The function is blocking for control loop time (optional)
     * @return True in case of success and False in case of problems.
     */
    virtual bool speedL(std::vector<double> zdDes, double maxAcceleration, double loopTime) = 0;

    /**
     * @brief Moves robot to desired joint position. Creats trajectory linear in joint space.
     * @param qDes [rad] Desired joint positions
     * @param speed [rad/s] Joint speed of leading axis
     * @param acceleration [rad/s^2] Joint acceleration of leading axis
     * @param async Specifying if the move command should be asynchronous. If asynchronous is
     *        true it is possible to stop a move command using either the stopJ or stopL
     *        function. Default is false, this means the function will block until the
     *        movement has completed.
     * @return True in case of success and False in case of problems.
     */
    virtual bool moveJ(std::vector<double> qDes, double speed, double acceleration, bool async) = 0;

    /**
     * @brief Moves robot to desired task position. Creats trajectory linear in task space.
     * @param zDes [rad] Desired task positions
     * @param speed [rad/s] End effector speed
     * @param acceleration [rad/s^2] End effector acceleration
     * @param async Specifying if the move command should be asynchronous. If asynchronous is
     *        true it is possible to stop a move command using either the stopJ or stopL
     *        function. Default is false, this means the function will block until the
     *        movement has completed.
     * @return True in case of success and False in case of problems.
     */
    virtual bool moveL(std::vector<double> zDes, double speed, double acceleration, bool async) = 0;

    /**
     * @brief Sets the gravity vector relative to the robot base frame. Used for calculation of
     *        internal robot dynamics to trigger the safety features.
     * @param gravity [m/s^2] Gravity vector, e.g. [9.81, 0, 0]
     * @return True in case of success and False in case of problems.
     */
    virtual bool setGravity(const std::vector<double>& gravity) = 0;

    /**
     * @brief Zeroes the TCP force/torque measurement from the builtin force/torque sensor by
     *        subtracting the current measurement from the subsequent.
     * @return True in case of success and False in case of problems.
     */
    virtual bool zeroFtSensor() = 0;

    /**
     * @brief Set robot to be controlled in force mode
     * @param forceFrame A pose vector that defines the force frame relative to the base frame
     * @param complianceSelector A 6d vector of 0s and 1s. 1 means that the robot will be compliant
     *        in the corresponding axis of the task frame
     * @param desiredForceTorque The forces/torques the robot will apply to its environment. The
     *        robot adjusts its position along/about compliant axis in order to achieve the
     *        specified force/torque. Values have no effect for non-compliant axes
     * @param type An integer [1;3] specifying how the robot interprets the force frame.
     *        1: The force frame is transformed in a way such that its y-axis is aligned with a
     *        vector pointing from the robot tcp towards the origin of the force frame.
     *        2: The force frame is not transformed.
     *        3: The force frame is transformed in a way such that its x-axis is the projection of
     *        the robot tcp velocity vector onto the x-y plane of the force frame
     * @param limits [6d vector] For compliant axes, these values are the maximum allowed tcp speed
     *        along/about the axis. For non-compliant axes, these values are the maximum allowed
     *        deviation along/about an axis between the actual tcp position and the one set by the
     *        program.
     * @return True in case of success and False in case of problems.
     */
    virtual bool forceMode(std::vector<double> forceFrame, std::vector<int> complianceSelector,
        std::vector<double> desiredForceTorque, int type, std::vector<double> limits) = 0;

    /**
     * @brief Requests the current joint torques of the robot
     * @return The joint torques
     */
    virtual std::vector<double> getJointForceTorques() = 0;

    /**
     * @brief Stop speed mode and decelerate the robot.
     * @return True in case of success and False in case of problems.
     */
    virtual bool speedStop() = 0;

    /**
     * @brief Stop servo mode and decelerate the robot.
     * @return True in case of success and False in case of problems.
     */
    virtual bool servoStop() = 0;

    /**
     * @brief Terminates the script running on the controller box.
     * @return void
     */
    virtual void stopScript() = 0;
};

}  // namespace crf::communication::universalrobotrtde
