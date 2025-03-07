/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Hannes Gamper CERN EN/SMM/MRO 2022
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>
#include <vector>
#include <utility>
#include <memory>

#include <nlohmann/json.hpp>

#include "KinematicChain/IKinematicChain.hpp"
#include "KinematicChain/URDFKinematicChain/URDFKinematicChain.hpp"

#include "ForwardKinematics/IForwardKinematics.hpp"
#include "ForwardKinematics/KinChainForwardKinematics/KinChainForwardKinematics.hpp"
#include "ForwardKinematics/MathExprForwardKinematics/MathExprForwardKinematics.hpp"

#include "Jacobian/IJacobian.hpp"
#include "Jacobian/KinChainJacobian/KinChainJacobian.hpp"
#include "Jacobian/MathExprJacobian/MathExprJacobian.hpp"

#include "EventLogger/EventLogger.hpp"
#include "Types/Types.hpp"

using crf::math::kinematicchain::IKinematicChain;
using crf::math::kinematicchain::URDFKinematicChain;
using crf::control::forwardkinematics::IForwardKinematics;
using crf::control::forwardkinematics::KinChainForwardKinematics;
using crf::control::forwardkinematics::MathExprForwardKinematics;
using crf::math::jacobian::IJacobian;
using crf::math::jacobian::MathExprJacobian;
using crf::math::jacobian::KinChainJacobian;
using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointAccelerations;
using crf::utility::types::JointForceTorques;
using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;
using crf::utility::types::TaskAcceleration;
using crf::utility::types::TaskForceTorque;
using crf::utility::types::TaskSpace;

namespace crf::actuators::robot {

/**
 * @ingroup group_robot
 * @brief Struct to group the joint limits of a robot. It includes values for max position,
 * min position, max velocity, max acceleration, and max torque.
 *
 */
struct JointLimits {
    JointPositions maxPosition;
    JointPositions minPosition;
    JointVelocities maxVelocity;
    JointAccelerations maxAcceleration;
    JointForceTorques maxTorque;

    /**
     * @brief Method to check if the values inside the struct share the same dimensions.
     * If they don't then an exception is thrown.
     *
     * @return uint64_t The size of all the joint values inside the struct.
     */
    uint64_t checkDimensions() const {
        uint64_t dimensions = maxPosition.size();
        if (minPosition.size() != dimensions ||
            maxVelocity.size() != dimensions ||
            maxAcceleration.size() != dimensions ||
            maxTorque.size() != dimensions) {
                throw std::invalid_argument("The size of joint limits does not match each other");
        }
        return dimensions;
    }
};

/**
 * @ingroup group_robot
 * @brief Struct to group the task limits of a robot. It includes values for max velocity, and
 * max acceleration.
 *
 */
struct TaskLimits {
    TaskVelocity maxVelocity;
    TaskAcceleration maxAcceleration;
};

/**
 * @ingroup group_robot
 * @brief Struct to group the profile parameters of a robot. It includes values for profile joint
 * velocity, profile joint acceleration, profile task velocity, and profile task acceleration.
 *
 */
struct ProfileParameters {
    JointVelocities jointVelocities;
    JointAccelerations jointAccelerations;
    TaskVelocity taskVelocity;
    TaskAcceleration taskAcceleration;

    /**
     * @brief Method to check if the values inside the struct share the same size.
     * If they don't then an exception is thrown.
     *
     * @return uint64_t The size of all the joint values inside the struct,
     * (task values are always six dimensional vectors)
     */
    uint64_t checkDimensions() const {
        uint64_t jointDim = jointVelocities.size();
        if (jointAccelerations.size() != jointDim) {
            throw std::invalid_argument(
                "The size of joint profile values do not match");
        }
        return jointDim;
    }
};

/**
 * @ingroup group_robot
 * @brief Struct to represent the lenghts of a robot. It separates the lengths between
 * lx, ly, and lz.
 *
 */
struct RobotLenghts {
    std::vector<double> lx;
    std::vector<double> ly;
    std::vector<double> lz;

    /**
     * @brief Method to check if the dimensions of the lenghts are correct
     *
     * @return uint64_t
     */
    uint64_t checkDimensions() const {
        uint64_t dimensions = lx.size();
        if (ly.size() != dimensions ||
            lz.size() != dimensions) {
            throw std::invalid_argument("The size of robot lengths does not match each other");
        }
        return dimensions;
    }
};

/**
 * @ingroup group_robot
 * @brief Class to parse a JSON object. The JSON object should contain a valid configuration
 * for a robot.
 *
 * @details The desired use of this class is to create an specific robot configuration class
 * that inherits from this one and adds extra parsing requirements. This class has the basic
 * parameters that any robot should have.
 *
 */
class RobotConfiguration {
 public:
    RobotConfiguration() = delete;
    explicit RobotConfiguration(const nlohmann::json& robotConfig);
    explicit RobotConfiguration(const std::string&) = delete;
    virtual ~RobotConfiguration() = default;

    /**
     * @brief Get the Joint Space DoF.
     *
     * @return uint64_t
     */
    uint64_t getJointSpaceDoF() const;
    /**
     * @brief Get the Task Space DoF.
     *
     * @return uint64_t
     */
    uint64_t getTaskSpaceDoF() const;
    /**
     * @brief Get the Robot Controller Loop Time.
     *
     * @return std::chrono::milliseconds
     */
    std::chrono::milliseconds getRobotControllerLoopTime() const;
    /**
     * @brief Get the Joint Limits object.
     *
     * @return JointLimits
     */
    JointLimits getJointLimits() const;
    /**
     * @brief Get the Task Limits object.
     *
     * @return TaskLimits
     */
    TaskLimits getTaskLimits() const;
    /**
     * @brief Get the Profile Parameters object.
     *
     * @return ProfileParameters
     */
    ProfileParameters getProfileParameters() const;
    /**
     * @brief Get the lengths of the robot components.
     * @details This parameter is optional in the JSON file. If it is
     * not provided, calling this method will throw an exception
     *
     * @return RobotLenghts
     */
    RobotLenghts getRobotLengths() const;
    /**
     * @brief Get the expression that defines the forward kinematics.
     * @details This parameter is optional in the JSON file. If it is
     * not provided, calling this method will throw an exception
     *
     * @return std::shared_ptr<IForwardKinematics>
     */
    std::shared_ptr<IForwardKinematics> getForwardKinematics() const;
    /**
     * @brief Get the Jacobian Math Expressions object.
     * @details This parameter is optional in the JSON file. If it is
     * not provided, calling this method will throw an exception
     *
     * @return std::shared_ptr<IJacobian>
     */
    std::shared_ptr<IJacobian> getJacobian() const;
    /**
     * @brief Get the Task Space object.
     * @details This parameter is optional in the JSON file. If it is
     * not provided, calling this method will return the default unreduced task space.
     *
     * @return TaskSpace
     */
    TaskSpace getTaskSpace() const;
    /**
     * @brief Get the validated JSON file given through the constructor
     * 
     * @return nlohmann::json 
     */
    nlohmann::json getConfigurationFile() const;

 protected:
    bool lenghtsParsed_;
    bool forwardKinematicsParsed_;
    bool jacobianParsed_;

 private:
    /**
     * @brief Method that parses the full JSON object. It is called in the
     * constructor of the class.
     * @details If the configuration file is wrong an exception will be thrown
     *
     * @param robotConfig JSON object to parse
     */
    void parse(const nlohmann::json& robotConfig);
    void parse(const std::string&) = delete;

    /**
     * @brief Method that parses the Joint Limits inside the JSON object. It is called
     * inside the parse method.
     * @details If the configuration file is wrong an exception will be thrown
     *
     * @param limits JSON object to parse the limits
     */
    void parseJointLimits(const nlohmann::json& limits);
    void parseJointLimits(const std::string&) = delete;

    /**
     * @brief Method that parses the Task Limits inside the JSON object. It is called
     * inside the parse method.
     * @details If the configuration file is wrong an exception will be thrown
     *
     * @param limits JSON object to parse the limits
     */
    void parseTaskLimits(const nlohmann::json& limits);
    void parseTaskLimits(const std::string&) = delete;

    /**
     * @brief Method that parses the Profile Parameters inside the JSON object. It is called
     * inside the parse method.
     * @details If the configuration file is wrong an exception will be thrown
     *
     * @param params JSON object to parse the Profile Parameters
     */
    void parseProfileParameters(const nlohmann::json& params);
    void parseProfileParameters(const std::string&) = delete;

    /**
     * @brief Method that parses the Robot Lenghts inside the JSON object. It is called
     * inside the parse method.
     * @details If the configuration file is wrong an exception will be thrown
     *
     * @param lengths JSON object to parse the Robot Lenghts
     */
    void parseExpressionLengths(const nlohmann::json& lengths);
    void parseExpressionLengths(const std::string&) = delete;

    /**
     * @brief Method that parses the Math Expressions inside the JSON object. It is called
     * inside the parse method.
     * @details If the configuration file is wrong an exception will be thrown
     *
     * @param expressions JSON object to parse the Math Expressions
     */
    void parseExpressionFK(const nlohmann::json& expressions);
    void parseExpressionFK(const std::string&) = delete;

    /**
     * @brief Method that parses the Jacobian inside the JSON object. It is called
     * inside the parse method.
     * @details If the configuration file is wrong an exception will be thrown
     *
     * @param jacobian JSON object to parse the Jacobian
     */
    void parseExpressionJacobian(const nlohmann::json& jacobian);
    void parseExpressionJacobian(const std::string&) = delete;

    /**
     * @brief Method that parses the Jacobian inside the JSON object. It is called
     * inside the parse method.
     * @details If the configuration file is wrong an exception will be thrown
     *
     * @param jacobian JSON object to parse the Jacobian
     */
    void parseKinChainJacobian(const nlohmann::json& URDF);
    void parseKinChainJacobian(const std::string&) = delete;

    /**
     * @brief Method that parses the Jacobian inside the JSON object. It is called
     * inside the parse method.
     * @details If the configuration file is wrong an exception will be thrown
     *
     * @param jacobian JSON object to parse the Jacobian
     */
    void parseKinChainFK(const nlohmann::json& URDF);
    void parseKinChainFK(const std::string&) = delete;

    nlohmann::json robotJSON_;
    uint64_t jointSpaceDoF_;
    uint64_t taskSpaceDoF_;
    std::chrono::milliseconds robotControllerLoopTime_;
    TaskLimits taskLimits_;
    JointLimits jointLimits_;
    ProfileParameters profileParameters_;
    RobotLenghts robotLengths_;
    TaskSpace taskSpace_;

    std::shared_ptr<IForwardKinematics> forwardKinematics_;
    std::shared_ptr<IJacobian> jacobian_;

    crf::utility::logger::EventLogger logger_;
};

}  // namespace crf::actuators::robot
