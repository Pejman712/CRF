/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2023
 *
 *  ================================================================================================================
 */

#pragma once

#include <map>
#include <string>
#include <vector>
#include <fstream>
#include <numeric>

#include "Types/Types.hpp"
#include "TrajectoryGenerator/ITaskTrajectoryGenerator.hpp"
#include "EventLogger/EventLogger.hpp"

using crf::math::rotation::OrientationRepresentation;
using crf::utility::types::eigenVectorFromStdVector;
using crf::math::rotation::quaternionFromArray;
using crf::math::rotation::CardanXYZ;
using crf::math::rotation::EulerZXZ;

namespace crf::control::trajectorygenerator {

/**
 * @brief It returns the trajectory stored in the csv file.
 *
 *        The first line of the file has to contain the titles of the corresponding data, following
 *        the order: 1st Pose, 2nd Velocity, 3rd Acceleration.
 *        There is no need for all 3 to be in the csv file.
 *
 *        The available titles for the POSE in the euclidean 6-dimensional space are:
 *         - PoseX,PoseY,PoseZ,PoseCardanX,PoseCardanY,PoseCardanZ
 *         - PoseX,PoseY,PoseZ,PoseEulerZ1,PoseEulerX,PoseEulerZ2
 *         - PoseX,PoseY,PoseZ,PoseAxisX,PoseAxisY,PoseAxisZ,PoseAngle
 *         - PoseX,PoseY,PoseZ,PoseQ1,PoseQ2,PoseQ3,PoseQ4
 *        The titles of the VELOCITIES in the 6-dimensional space are:
 *         - VelX,VelY,VelZ,VelAngularX,VelAngularY,VelAngularZ
 *        The titles of the ACCELERATIONS in the 6-dimensional space are:
 *         - AccX,AccY,AccZ,AccAngularX,AccAngularY,AccAngularZ
 *
 *        For task spaces with customized number of dimensions the titles are PosCustom, VelCustom
 *        and AccCustom.
 *        For example, in a custom 2-dimensional task space, the titles would be:
 *        - PosCustom1,PosCustom2,VelCustom1,VelCustom2,AccCustom1,AccCustom2
 *
 *        Each line of the file has to correspond to the data in one step time.
 *        The step, the difference between each time, is the cycleTime.
 */
class PreplannedTaskTrajectory: public ITaskTrajectoryGenerator {
 public:
    PreplannedTaskTrajectory(const std::string& trajectoryFile, const double& cycleTime);
    ~PreplannedTaskTrajectory() override;

    void setInitialPose(const TaskPose& initialPose);
    void append(const std::vector<TaskPose>& path) override;
    void setProfileVelocity(const TaskVelocity& vel) override;
    void setProfileAcceleration(const TaskAcceleration& acc) override;
    void reset() override;
    void clearMemory() override;
    bool isTrajectoryRunning() override;
    TaskSignals getTrajectoryPoint(double Tp) override;

 private:
    crf::utility::logger::EventLogger logger_;
    std::ifstream file_;
    double cycleTime_;
    uint64_t index_;
    bool is6DTaskSpace_;
    std::array<bool, 3> content_;  // [isTherePosition, isThereVelocity, isThereAcceleration]
    uint64_t orientationType_;
    std::vector<TaskPose> taskPose_;
    std::vector<TaskVelocity> taskVelocity_;
    std::vector<TaskAcceleration> taskAcceleration_;
    uint64_t cycle_;
    uint64_t quantityOfCycles_;

    const std::vector<std::vector<std::string>> taskSpace6D_{
        {"PoseX", "PoseY", "PoseZ"},
        {"VelX", "VelY", "VelZ", "VelAngularX", "VelAngularY", "VelAngularZ"},
        {"AccX", "AccY", "AccZ", "AccAngularX", "AccAngularY", "AccAngularZ"}
    };
    const std::vector<std::vector<std::string>> orientations_{
        {"PoseQ1", "PoseQ2", "PoseQ3", "PoseQ4"},
        {"PoseAngle", "PoseAxisX", "PoseAxisY", "PoseAxisZ"},
        {"PoseCardanX", "PoseCardanY", "PoseCardanZ"},
        {"PoseEulerZ1", "PoseEulerX", "PoseEulerZ2"}
    };

    bool matchHeader(const std::vector<std::string>& fileHeader,
        const std::vector<std::string>& referenceHeader);
    bool matchHeaderWIndex(const std::vector<std::string>& fileHeader,
        const std::vector<std::string>& referenceHeader);
};

}  // namespace crf::control::trajectorygenerator
