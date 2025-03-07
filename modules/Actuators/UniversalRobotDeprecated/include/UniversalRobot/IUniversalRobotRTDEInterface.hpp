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

namespace crf::actuators::universalrobot {

class IUniversalRobotRTDEInterface {
 public:
    virtual ~IUniversalRobotRTDEInterface() = default;

    // Interfaces for rtdeReceive
    virtual void initRtdeReceiveInterface(std::string IP) = 0;
    virtual std::vector<double> getActualQ() = 0;
    virtual std::vector<double> getActualQd() = 0;
    virtual std::vector<double> getActualTCPPose() = 0;

    // Interfaces for rtdeControl
    virtual void initRtdeControlInterface(std::string IP) = 0;
    virtual bool servoJ(std::vector<double> qDes, double maxVel, double maxAcc, double loopTime,
        double lookAheadTime, double gain) = 0;
    virtual bool speedJ(std::vector<double> qdDes, double maxAcceleration, double loopTime) = 0;
    virtual bool speedStop() = 0;
    virtual bool servoStop() = 0;
    virtual void stopScript() = 0;
};

}  // namespace crf::actuators::universalrobot
