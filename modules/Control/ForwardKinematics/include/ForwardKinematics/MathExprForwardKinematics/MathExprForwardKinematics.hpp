/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2021
 *
 *  ================================================================================================================
*/

#pragma once

#include <optional>
#include <string>
#include <vector>

#include "EventLogger/EventLogger.hpp"
#include "MathExprTk/exprtk.hpp"
#include "nlohmann/json.hpp"
#include "ForwardKinematics/IForwardKinematics.hpp"
#include "Types/Types.hpp"

using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointAccelerations;
using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;
using crf::utility::types::TaskAcceleration;
using crf::math::rotation::OrientationRepresentation;

namespace crf::control::forwardkinematics {

/**
 * @ingroup group_math_expressions_forward_kinematics
 * @brief It return the end-effector position, velocity and/or acceleration from specified values
 *        of the joint positions, velocities and accelerations; reading the mathematical forward
 *        kinematic expressions of a specific robot from an external source and evaluating them
 *        with the specified given values of the joint parameters
 *
 * @param jsonConfig is the container of the mathematical forward kinematic expressions
 * @param lxValues is the vector of lengths between the robot joints in the X axis
 * @param lyValues is the vector of lengths between the robot joints in the Y axis
 * @param lzValues is the vector of lengths between the robot joints in the Z axis
 */

class MathExprForwardKinematics : public IForwardKinematics {
 public:
    MathExprForwardKinematics(nlohmann::json jsonConfig,
        std::vector<double> lx,
        std::vector<double> ly,
        std::vector<double> lz);
    ~MathExprForwardKinematics() override;

    std::optional<TaskPose> getPose(const JointPositions& jointPositions) override;
    std::optional<TaskVelocity> getVelocity(
        const JointPositions& jointPositions,
        const JointVelocities& jointVelocities) override;
    std::optional<TaskAcceleration> getAcceleration(
        const JointPositions& jointPositions,
        const JointVelocities& jointVelocities,
        const JointAccelerations& jointAccelerations) override;

 private:
    utility::logger::EventLogger logger_;
    nlohmann::json jsonConfig_;
    std::vector<double> lx_;
    std::vector<double> ly_;
    std::vector<double> lz_;
    unsigned int numberOfJoints_;
    std::vector<double> q_;
    std::vector<double> qd_;
    std::vector<double> qdd_;
    std::vector<double> RIE_;
    std::vector<double> quaternion_;
    std::vector<double> cardanXYZ_;
    std::vector<double> IrIE_;
    std::vector<double> IwIE_;
    std::vector<double> IvIE_;
    std::vector<double> IalphaIE_;
    std::vector<double> IaIE_;
    exprtk::symbol_table<double> symbolTable_;
    exprtk::expression<double> poseExpression_;
    exprtk::expression<double> velocityExpression_;
    exprtk::expression<double> accelerationExpression_;
    uint64_t sizeJSONFile_;
    bool isTherePosition_;
    OrientationRepresentation orientationRepresentation_;
    bool isThereVelocity_;
    bool isThereAcceleration_;

    /**
     * @brief Parses the mathematical expressions from a JSON file according to the defined grammar
     *        and internally builds parse trees. This function is called in the constructor.
     *
     * @param expression is the forward kinematics expression in exprtk readable format.
     * @param linear is a string that contains the name of the linear part of the forward
     *        kinematics that obtains a non custom element. It can be "IrIE" for the position,
     *        "IvIE" for the linear velocity, and "IaIE" for the linear acceleration.
     * @param angular is a string that contains the name of the angular part of the forward
     *        kinematics that obtains a non custom element. It can be "RIE" or "Quaternion" for
     *        the orientation, "IwIE" for the angular velocity, and "IalphaIE" for the angular
     *        acceleration
     * @return The parsed mathematical expressions.
     * @return std::nullopt when the linear or angular forward kinematics expression that is being
     *         parsed is not in the JSON file.
     */
    std::optional<exprtk::expression<double>> parse(
        exprtk::expression<double> expression,
        const std::string linear,
        const std::string angular);
};

std::string ifFromNaN(const std::string& input);

}  // namespace crf::control::forwardkinematics
