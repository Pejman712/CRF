/* © Copyright CERN 2022. All rights reserved. This software is released under a
 * CERN proprietary software license. Any permission to use it shall be granted
 * in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Ante Marić CERN BE/CEM/MRO 2022
 *          Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ================================================================================================================
 */

#pragma once

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "EventLogger/EventLogger.hpp"
#include "KinematicChain/IKinematicChain.hpp"
#include "Types/Types.hpp"

namespace crf::math::kinematicchain {

/**
 * @ingroup group_urdf_kinematic_chain
 * @brief Class that inherits from IKinematic Chain. This class generates
 * a Kinematic Chain object trough a URDF file.
 *
 */
class URDFKinematicChain : public IKinematicChain {
 public:
    /**
     * @brief Parses and constructs a kinematic chain from a URDF file for
     * specified end-effector
     * @param pathToRobotURDF Path to URDF file
     * @param endEffectorName Name of link, which parent joint's reference frame
     * will be end-effector point
     */
    URDFKinematicChain(
        const std::string& pathToRobotURDF,
        const std::string& endEffectorName = "leafLink",
        const std::string& pathToToolURDF = "");
    ~URDFKinematicChain() override;

    /**
     * @brief returns the type of URDF description (Tool, Platform, Arm, or
     * Combined)
     * @return description_type_ - type of URDF description
     */
    DescriptionType getType() override;

    /**
     * @brief returns the type of joint at specified index
     * @param i represents index of specified joint
     * @return jointType - type of specified joint
     * 0 for UNKNOWN, 1 for REVOLUTE, 2 for CONTINUOUS, 3 for PRISMATIC
     */
    int getJointType(const int& jointIndex) override;

    /**
     * @brief returns the size of the kinematic chain
     * @return chainSize_ - number of joints in the kinematic chain
     */
    size_t getChainSize() const override;

    /**
     * @brief returns the number of platform wheels
     * @return numWheels_ - number of platform wheels (0 if platform is not
     * present)
     */
    size_t getNumWheels() const override;

    /**
     * @brief Returns the wheel radius of the mobile platform
     * @returns wheelRadius_ - wheel radius
     */
    double getWheelRadius() override;

    /**
     * @brief Returns longitudinal wheel distance
     * i.e, distance from the shaft of the front left wheel to the back left wheel
     * @return platformLength_ - longitudinal wheel distance
     */
    double getPlatformL() override;

    /**
     * @brief Returns lateral wheel distance
     * i.e, distance from the shaft of the front left wheel to the front right
     * wheel
     * @return platformWidth_ - lateral wheel distance
     */
    double getPlatformW() override;

    /**
     * @brief Sets joints to positions from the argument and calculates
     * accordingly all parameters such as rotations and translations
     * @param jointPositions Configuration containing all joint angles
     */
    void setJointPositions(const crf::utility::types::JointPositions& jointPositions) override;

    /**
     * @brief Returns requested axis.
     * @param axis Code for a particular axis.
     * @param jointOrWheelN Joint or Wheel number, indexed from 0.
     * Joint are numbered from the base to the end effector,
     * Wheels are numbered couterclockwise, starting from the front right.
     * @return Axis specified by the parameter "axis".
     * It is returned as Eigen::Vector3d.
     */
    Eigen::Vector3d getAxis(const Axes axis, const int& jointOrWheelN) override;

    /**
     * @brief Returns requested translation.
     * @param translation Code for a particular translation.
     * @param jointN Joint number, indexed from 0
     * @return Translation described by the parameter translation. It is returned
     * as Eigen::Vector3d.
     */
    Eigen::Vector3d getTranslation(const Translations translation, const int& jointN) override;

    /**
     * @brief Returns requested rotation.
     * @param rotation Code for a particular rotation.
     * @param jointN Joint number, indexed from 0
     * @return Rotation described by the parameter rotation. It is returned as
     * Eigen::Quaterniond.
     */
    Eigen::Quaterniond getRotation(const Rotations rotation, const int& jointN) override;

    /**
     * @brief Returns requested axis.
     * @param axis Code for a particular axis.
     * @param jointOrWheelN Joint or Wheel number, indexed from 0.
     * Joint are numbered from the base to the end effector,
     * Wheels are numbered couterclockwise, starting from the front right.
     * @param jointPositions Configuration containing all joint angles
     * @return Axis specified by the parameter "axis".
     * It is returned as Eigen::Vector3d.
     */
    Eigen::Vector3d computeAxis(
        const Axes axis,
        const int& jointOrWheelN,
        const crf::utility::types::JointPositions& jointPositions) override;

    /**
     * @brief Computes geometric data based on the jointPositions
     * and returns requested translation.
     * @param translation Code for a particular translation.
     * @param jointN Joint number, indexed from 0
     * @param jointPositions Configuration containing all joint angles
     * @return Translation described by the parameter translation. It is returned
     * as Eigen::Vector3d.
     */
    Eigen::Vector3d computeTranslation(
        const Translations translation,
        const int& jointN,
        const crf::utility::types::JointPositions& jointPositions) override;

    /**
     * @brief Computes geometric data based on the jointPositions
     * and returns requested rotation.
     * @param rotation Code for a particular rotation.
     * @param jointN Joint number, indexed from 0
     * @param jointPositions Configuration containing all joint angles
     * @return Rotation described by the parameter rotation. It is returned as
     * Eigen::Quaterniond.
     */
    Eigen::Quaterniond computeRotation(
        const Rotations rotation,
        const int& jointN,
        const crf::utility::types::JointPositions& jointPositions) override;

 protected:
    /**
     * @brief Parses tool URDF and updates accordingly geometric information of the arm.
     */
    void parseToolURDF(const std::string& pathToToolURDF);

    /**
     * @brief Calculates longitudinal and lateral distance between wheel shafts of
     * mobile platform
     */
    void parsePlatform();

    utility::logger::EventLogger logger_;

    /**
     * @brief Parsed urdf model object.
     */
    std::shared_ptr<urdf::ModelInterface> model_;

    /**
     * @brief Name of specified end effector.
     */
    std::string endEffectorName_;

    /**
     * @brief Type of URDF description.
     */
    DescriptionType descriptionType_;

    /**
     * @brief Pointers to joint objects representing the kinematic chain of the
     * arm id present. Otherwise an empty std::vector.
     */
    std::vector<std::shared_ptr<urdf::Joint>> joints_;

    /**
     * @brief Pointers to joint objects representing wheel drives if pressent.
     * Otherwise an empty std::vector.
     */
    std::vector<std::shared_ptr<urdf::Joint>> wheelDrives_;

    /**
     * @brief Number of the wheels.
     */
    int noOfWheels_;

    /**
     * @brief Radius of first detected wheel (assumes same radius for all wheels).
     */
    double wheelRadius_;

    /**
     * @brief Longitudinal wheel distance (front - back).
     */
    double platformLength_;

    /**
     * @brief Lateral wheel distance (left - right).
     */
    double platformWidth_;

    /**
     * @brief Number of joints in chain.
     */
    size_t chainSize_;

    /**
     * @brief Tells, whether, there was an inicialisation of
     * joints positions to vector of zeros and calculates
     * accordingly all parameters such as axises, translations and rotations.
     */
    bool initialisedGeometricData_ = false;

    /**
     * @brief Wheels axes expressed in the inertial frame.
     */
    std::vector<Eigen::Vector3d> In_W_;

    /**
     * @brief Wheels axes expressed in the wheels frame.
     */
    std::vector<Eigen::Vector3d> Wn_W_;

    /**
     * @brief Translations between inertial frame and the wheels.
     */
    std::vector<Eigen::Vector3d> Ir_IW_;

    /**
     * @brief Rotation between inertial frame and the wheels.
     */
    std::vector<Eigen::Quaterniond> IR_IWZeroPosition_;

    /**
     * @brief Joint axes expressed in the joints frame.
     */
    std::vector<Eigen::Vector3d> Jn_J_;

    /**
     * @brief Translations from parent joint to child joint (ith joint) in zero
     * position, expressed in the parent joint frame for the zeroth joint it is a
     * translation from the inertial frame expressed in the inertial frame in zero
     * position.
     */
    std::vector<Eigen::Vector3d> Pr_PJZeroPosition_;

    /**
     * @brief Rotations from parent joint to child joint (ith joint) in zero
     * position, expressed in the parent joint frame for the zeroth joint it is a
     * trotation from the inertial frame expressed in the inertial frame in zero
     * position.
     */
    std::vector<Eigen::Quaterniond> PR_PJZeroPosition_;

    /**
     * @brief Vector from last joint to end-effector
     * expressed in the frame of the last joint.
     */
    Eigen::Vector3d Lr_LE_;

    /**
     * @brief Rotation from last joint to end-effector
     * expressed in the frame of the last joint.
     */
    Eigen::Quaterniond LR_LE_;

    /**
     * @brief Translations from parent joint to the indexed joint, expressed in
     * the inertial frame, for the zeroth joint it is translation from the base
     * frame to the zeroth joint, expressed in the inertial frame.
     */
    std::vector<Eigen::Vector3d> Ir_PJ_;

    /**
     * @brief Translations from parent joint to the indexed joint, expressed in
     * the parent joint frame, for the zeroth joint it is translation from the
     * base frame to the zeroth joint, expressed in the inertial frame.
     */
    std::vector<Eigen::Vector3d> Pr_PJ_;

    /**
     * @brief Rotations from parent joint to child joint (ith joint), expressed in
     * the parent joint frame, for the zeroth joint it is rotation from the
     * inertial frame to the zeroth joint, expressed in the inertial frame.
     */
    std::vector<Eigen::Quaterniond> PR_PJ_;

    /**
     * @brief Translation from the inertial frame to the indexed joint.
     */
    std::vector<Eigen::Vector3d> Ir_IJ_;

    /**
     * @brief Rotation from the inertial frame to the indexed joint.
     */
    std::vector<Eigen::Quaterniond> IR_IJ_;

    /**
     * @brief Joints axes expressed in the inertial frame.
     */
    std::vector<Eigen::Vector3d> In_J_;

    /**
     * @brief Translation from the indexed joint to the end effector,
     * expressed in the indexed joints frame.
     */
    std::vector<Eigen::Vector3d> Jr_JE_;

    /**
     * @brief Translation from the indexed joint to the end effector,
     * expressed in the inertial frame.
     */
    std::vector<Eigen::Vector3d> Ir_JE_;

    /**
     * @brief Translation from the inertial frame to the end effector,
     * expressed in the inertial frame.
     */
    Eigen::Vector3d Ir_IE_;

    /**
     * @brief Rotation from initial frame to the end effector,
     * expressed in the inertial frame.
     */
    Eigen::Quaterniond IR_IE_;

    /**
     * @brief Current joint angles.
     */
    utility::types::JointPositions jointPositions_;
};

}  // namespace crf::math::kinematicchain
