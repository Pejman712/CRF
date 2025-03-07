/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *         Laura Rodrigo Pérez CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#pragma once

#include <chrono>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "EventLogger/EventLogger.hpp"
#include "Types/Types.hpp"

namespace crf::actuators::robotarm {

struct DHParameter {
    enum JointType { Rotational, Linear };

    double a;
    double alpha;
    double d;
    double theta;
    JointType type;
};

struct JointLimits {
    double maximumPosition;
    double minimumPosition;
    double maximumVelocity;
    double maximumAcceleration;
    double maximumTorque;
};

struct TaskLimits {
    crf::utility::types::TaskVelocity maximumVelocity;
    crf::utility::types::TaskAcceleration maximumAcceleration;
};

struct parametersPID {
    std::vector<double> KpJoint;
    std::vector<double> KiJoint;
    std::vector<double> KdJoint;

    std::vector<double> KpTask;
    std::vector<double> KiTask;
    std::vector<double> KdTask;
};

class RobotArmConfiguration {
 public:
    RobotArmConfiguration();
    virtual ~RobotArmConfiguration() = default;
    /*
     * @brief
     * @param
     * @return
     * @return
     */
    virtual bool parse(const nlohmann::json& robotJSON);
    bool parse(const std::string&) = delete;
    /*
     * @brief
     * @return
     * @return
     */
    std::vector<JointLimits> getJointsConfiguration();
    /*
     * @brief
     * @return
     * @return
     */
    std::vector<int> getJointsDirection();
    /*
     * @brief
     * @return
     * @return
     */
    std::vector<double> getJointsOffset();
    /*
     * @brief
     * @return
     * @return
     */
    std::vector<DHParameter> getKinematicChain();
    /*
     * @brief
     * @return
     * @return
     */
    TaskLimits getTaskLimits();
    /*
     * @brief
     * @return
     * @return
     */
    unsigned int getNumberOfJoints();
    /*
     * @brief
     * @return
     * @return
     */
    std::chrono::milliseconds getRTLoopTime();
    /*
     * @brief
     * @return
     * @return
     */
    parametersPID getParametersPIDs();
    /*
     * @brief Obtain a vector with size 3 that represents the length in x, y and z axis.
     *        In each direction there is a vector with the size equal to the number of joints that
     *        represent the lengths of the links.
     *
     * @return vectors obtained
     */
    std::array<std::vector<double>, 3> getRobotLengths();
    /*
     * @brief Obtain the forward kinematics mathematical expressions
     *
     * @return json variable with the forward kinematics mathematical expressions
     */
    nlohmann::json getFKMathExpressions();
    /*
     * @brief Obtain the jacobian matrix mathematical expressions
     *
     * @return json variable with the jacobian matrix mathematical expressions
     */
    nlohmann::json getJacobianMathExpressions();

 protected:
    crf::utility::logger::EventLogger logger_;
    std::chrono::milliseconds rtLoopTime_;
    std::vector<DHParameter> kinematicChain_;
    std::vector<JointLimits> joints_;
    std::vector<int> jointsDirection_;
    std::vector<double> jointsOffset_;
    TaskLimits limits_;
    parametersPID pids_;
    std::array<std::vector<double>, 3> robotLengths_;
    nlohmann::json fkJson_;
    nlohmann::json jacobianJson_;

    virtual void cleanup();
};

}  // namespace crf::actuators::robotarm
