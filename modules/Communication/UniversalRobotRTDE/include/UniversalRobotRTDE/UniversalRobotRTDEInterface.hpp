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
#include <memory>

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include "UniversalRobotRTDE/IUniversalRobotRTDEInterface.hpp"

namespace crf::communication::universalrobotrtde {

class UniversalRobotRTDEInterface: public IUniversalRobotRTDEInterface {
 public:
    ~UniversalRobotRTDEInterface() override = default;

    void initRtdeReceiveInterface(std::string IP) override;
    std::vector<double> getActualQ() override;
    std::vector<double> getActualQd() override;
    std::vector<double> getActualTCPPose() override;
    std::vector<double> getActualTCPSpeed() override;
    std::vector<double> getActualTCPForce() override;
    uint32_t getRobotStatus() override;
    int32_t getRobotMode() override;
    uint32_t getSafetyStatusBits() override;

    void initRtdeControlInterface(std::string IP) override;
    bool servoJ(std::vector<double> qDes, double maxVel, double maxAcc, double loopTime,
        double lookAheadTime, double gain) override;
    bool speedJ(std::vector<double> qdDes, double maxAcceleration, double loopTime) override;
    bool speedL(std::vector<double> zdDes, double maxAcceleration, double loopTime) override;
    bool moveJ(std::vector<double> qDes, double speed, double acceleration, bool async) override;
    bool moveL(std::vector<double> zDes, double speed, double acceleration, bool async) override;
    bool setGravity(const std::vector<double>& gravity) override;
    bool zeroFtSensor() override;
    bool forceMode(std::vector<double> forceFrame, std::vector<int> complianceSelector,
        std::vector<double> desiredForceTorque, int type, std::vector<double> limits) override;
    std::vector<double> getJointForceTorques() override;
    bool speedStop() override;
    bool servoStop() override;
    void stopScript() override;

 private:
    std::unique_ptr<ur_rtde::RTDEControlInterface> rtdeControl_;
    std::unique_ptr<ur_rtde::RTDEReceiveInterface> rtdeReceive_;
};

}  // namespace crf::communication::universalrobotrtde
