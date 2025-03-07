/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#pragma once

#include <vector>
#include <map>
#include <mutex>
#include <atomic>

#include "Types/Signals.hpp"
#include "GeometricMethods/CubicPolynomial/CubicPolynomial.hpp"
#include "TrajectoryGenerator/IJointTrajectoryGenerator.hpp"
#include "EventLogger/EventLogger.hpp"

using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointAccelerations;
using crf::utility::types::JointForceTorques;
using crf::utility::types::JointSignals;

namespace crf::control::trajectorygenerator {

/**
 * @brief Implementation of IJointTrajectoryGenerator. Cubic polinomial trajectory
 * with a set of appended waypoints.
 * @details This class is thread safe
 *
 */
class CubicJointsTrajectory: public IJointTrajectoryGenerator {
 public:
    CubicJointsTrajectory(
        const JointVelocities& maxVelocity, const JointAccelerations& maxAcceleration);
    ~CubicJointsTrajectory() override;

    void setInitialPosition(const JointPositions& initialPosition) override;
    void append(const std::vector<JointPositions>& path) override;
    void setProfileVelocity(const JointVelocities& vel) override;
    void setProfileAcceleration(const JointAccelerations& acc) override;
    void reset() override;
    void clearMemory() override;
    bool isTrajectoryRunning() override;
    JointSignals getTrajectoryPoint(double Tp) override;

 private:
    JointVelocities maxVelocity_;
    JointAccelerations maxAcceleration_;

    JointPositions initialPosition_;
    std::vector<JointPositions> pathPoints_;
    std::vector<double> ranges_;
    std::atomic<bool> firstTrajectory_;
    std::atomic<bool> initialPositionSet_;
    uint64_t dimensions_;

    std::mutex mtx_;
    std::mutex profileValuesMtx_;

    std::map<double, std::vector<crf::math::geometricmethods::CubicPolynomial>> trajectories_;
    std::vector<crf::math::geometricmethods::CubicPolynomial> currentTrajectory_;
    std::atomic<double> lastEvaluationPoint_;
    std::atomic<double> endTrajectoryRange_;

    crf::utility::logger::EventLogger logger_;
};

}  // namespace crf::control::trajectorygenerator
