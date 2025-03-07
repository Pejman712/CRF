/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN EN/SMM/MRO
 * Contributor: Francesco Riccardi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <map>
#include <string>
#include <vector>

#include <boost/assign/list_inserter.hpp>
#include <nlohmann/json.hpp>

#include "TIMArm/TIMArmConfiguration.hpp"
#include "RobotArm/RobotArmConfiguration.hpp"

namespace crf::actuators::timarm {

TIMArmConfiguration::TIMArmConfiguration() :
    RobotArmConfiguration(),
    kinovaSerialNumber_(),
    kinovaJointsNumber_(),
    kinovaNetworkConfiguration_(),
    kinovaArmJSON_(),
    etherCATArmJSON_() {
    logger_->debug("CTor");
    boost::assign::insert(mapJointType_)
        (crf::actuators::robotarm::DHParameter::JointType::Rotational, "Rotational")
        (crf::actuators::robotarm::DHParameter::JointType::Linear, "Linear");
}

bool TIMArmConfiguration::parse(const nlohmann::json& robotJSON) {
    logger_->debug("parse");
    cleanup();
    if (!RobotArmConfiguration::parse(robotJSON)) {
        return false;
    }
    try {
        kinovaSerialNumber_ = robotJSON.at("KinovaArm").at("SerialNumber").get<std::string>();
        kinovaJointsNumber_ = robotJSON["KinovaArm"]["NumberOfJoints"].get<unsigned int>();
        etherCATJointsNumber_ = robotJSON["EtherCATRobotArm"]["NumberOfJoints"].get<unsigned int>();
        if (getNumberOfJoints() != kinovaJointsNumber_ + etherCATJointsNumber_) {
            logger_->warn("The number of joints does not match");
            cleanup();
            return false;
        }

        nlohmann::json eth = robotJSON["KinovaArm"]["EthernetCommunication"];
        kinovaNetworkConfiguration_.robotAddressIP = eth["IPAdress"].get<std::string>();
        kinovaNetworkConfiguration_.subnetMask = eth["SubnetMask"].get<std::string>();
        kinovaNetworkConfiguration_.port = eth["Port"].get<unsigned int>();
        kinovaNetworkConfiguration_.localAddressIP = eth["LocalPcIPAddress"].get<std::string>();
    } catch (std::exception& e) {
        logger_->warn("Failed to parse because: {}", e.what());
        cleanup();
        return false;
    }
    generateKinovaArmJSON();
    generateHarmonicArmJSON();
    return true;
}

std::string TIMArmConfiguration::getKinovaSerialNumber() {
    return kinovaSerialNumber_;
}

unsigned int TIMArmConfiguration::getKinovaNumberOfJoints() {
    return kinovaJointsNumber_;
}

unsigned int TIMArmConfiguration::getHarmonicNumberOfJoints() {
    return etherCATJointsNumber_;
}

crf::actuators::kinovaarm::KinovaJacoNetworkConfiguration TIMArmConfiguration::getKinovaNetworkConfiguration() { //  NOLINT
    return kinovaNetworkConfiguration_;
}

nlohmann::json TIMArmConfiguration::getKinovaArmJSON() {
    return kinovaArmJSON_;
}

nlohmann::json TIMArmConfiguration::getHarmonicArmJSON() {
    return etherCATArmJSON_;
}

void TIMArmConfiguration::cleanup() {
    logger_->debug("cleanup");
    RobotArmConfiguration::cleanup();
    kinovaSerialNumber_ = "";
    kinovaJointsNumber_ = 0;
    etherCATJointsNumber_ = 0;
    kinovaNetworkConfiguration_ = crf::actuators::kinovaarm::KinovaJacoNetworkConfiguration();
    kinovaArmJSON_.clear();
    etherCATArmJSON_.clear();
}

void TIMArmConfiguration::generateKinovaArmJSON() {
    logger_->debug("generateKinovaArmJSON");
    kinovaArmJSON_.clear();
    kinovaArmJSON_["SerialNumber"] = kinovaSerialNumber_;
    kinovaArmJSON_["EthernetCommunication"] = {
        {"IPAdress", kinovaNetworkConfiguration_.robotAddressIP},
        {"SubnetMask", kinovaNetworkConfiguration_.subnetMask},
        {"Port", kinovaNetworkConfiguration_.port},
        {"LocalPcIPAddress", kinovaNetworkConfiguration_.localAddressIP}
    };
    kinovaArmJSON_["NumberOfJoints"] = kinovaJointsNumber_;
    kinovaArmJSON_["LoopTimeMs"] = RobotArmConfiguration::getRTLoopTime().count();
    nlohmann::json dhHolder;
    nlohmann::json jointsLimitsHolder;
    std::vector<crf::actuators::robotarm::DHParameter> dhParameters = getKinematicChain();
    std::vector<crf::actuators::robotarm::JointLimits> jointsLimits = getJointsConfiguration();

    crf::actuators::robotarm::parametersPID parametersPID =
        RobotArmConfiguration::getParametersPIDs();

    nlohmann::json JointKpHolder;
    nlohmann::json JointKiHolder;
    nlohmann::json JointKdHolder;
    for (unsigned int i=3; i < getNumberOfJoints(); i++) {
        dhHolder.push_back({
            {"D", dhParameters[i].d},
            {"Theta", dhParameters[i].theta},
            {"A", dhParameters[i].a},
            {"Alpha", dhParameters[i].alpha},
            {"Type", mapJointType_[dhParameters[i].type]}
        });
        jointsLimitsHolder.push_back({
            {"Maximum", jointsLimits[i].maximumPosition},
            {"Minimum", jointsLimits[i].minimumPosition},
            {"MaximumVelocity", jointsLimits[i].maximumVelocity},
            {"MaximumAcceleration", jointsLimits[i].maximumAcceleration},
            {"MaximumTorque", jointsLimits[i].maximumTorque},
            {"Direction", jointsDirection_[i]},
            {"Offset", jointsOffset_[i]},
            {"Kp", parametersPID.KpJoint[i]},
            {"Ki", parametersPID.KiJoint[i]},
            {"Kd", parametersPID.KdJoint[i]}
        });
    }
    kinovaArmJSON_["KinematicChain"] = {
        {"format", "DH"},
        {"parameters", dhHolder}
    };
    kinovaArmJSON_["Joints"] = jointsLimitsHolder;
    crf::actuators::robotarm::TaskLimits taskLimits =
        RobotArmConfiguration::getTaskLimits();
    nlohmann::json taskMaximumVelocityHolder;
    nlohmann::json taskMaximumAccelerationHolder;
    nlohmann::json TaskKpHolder;
    nlohmann::json TaskKiHolder;
    nlohmann::json TaskKdHolder;
    for (unsigned int i=0; i < 6; i++) {
        taskMaximumVelocityHolder.push_back(taskLimits.maximumVelocity[i]);
        taskMaximumAccelerationHolder.push_back(taskLimits.maximumAcceleration[i]);
        TaskKpHolder.push_back(parametersPID.KpTask[i]);
        TaskKiHolder.push_back(parametersPID.KiTask[i]);
        TaskKdHolder.push_back(parametersPID.KdTask[i]);
    }
    kinovaArmJSON_["Task"] = {
        {"MaximumVelocity", taskMaximumVelocityHolder},
        {"MaximumAcceleration", taskMaximumAccelerationHolder},
        {"Kp", TaskKpHolder},
        {"Ki", TaskKiHolder},
        {"Kd", TaskKdHolder}
    };
}

void TIMArmConfiguration::generateHarmonicArmJSON() {
    logger_->debug("generateHarmonicArmJSON");
    etherCATArmJSON_.clear();
    etherCATArmJSON_["NumberOfJoints"] = etherCATJointsNumber_;
    etherCATArmJSON_["LoopTimeMs"] = RobotArmConfiguration::getRTLoopTime().count();
    nlohmann::json dhHolder;
    nlohmann::json jointsLimitsHolder;
    std::vector<crf::actuators::robotarm::DHParameter> dhParameters = getKinematicChain();
    std::vector<crf::actuators::robotarm::JointLimits> jointsLimits = getJointsConfiguration();
    crf::actuators::robotarm::parametersPID parametersPID =
        RobotArmConfiguration::getParametersPIDs();
    for (unsigned int i=0; i < etherCATJointsNumber_; i++) {
        dhHolder.push_back({
            {"D", dhParameters[i].d},
            {"Theta", dhParameters[i].theta},
            {"A", dhParameters[i].a},
            {"Alpha", dhParameters[i].alpha},
            {"Type", mapJointType_[dhParameters[i].type]}
        });
        jointsLimitsHolder.push_back({
            {"Maximum", jointsLimits[i].maximumPosition},
            {"Minimum", jointsLimits[i].minimumPosition},
            {"MaximumVelocity", jointsLimits[i].maximumVelocity},
            {"MaximumAcceleration", jointsLimits[i].maximumAcceleration},
            {"MaximumTorque", jointsLimits[i].maximumTorque},
            {"Direction", jointsDirection_[i]},
            {"Offset", jointsOffset_[i]},
            {"Kp", parametersPID.KpJoint[i]},
            {"Ki", parametersPID.KiJoint[i]},
            {"Kd", parametersPID.KdJoint[i]}
        });
    }
    etherCATArmJSON_["KinematicChain"] = {
        {"format", "DH"},
        {"parameters", dhHolder}
    };
    etherCATArmJSON_["Joints"] = jointsLimitsHolder;
    crf::actuators::robotarm::TaskLimits taskLimits =
        RobotArmConfiguration::getTaskLimits();
    nlohmann::json taskMaximumVelocityHolder;
    nlohmann::json taskMaximumAccelerationHolder;
    nlohmann::json TaskKpHolder;
    nlohmann::json TaskKiHolder;
    nlohmann::json TaskKdHolder;
    for (unsigned int i=0; i < 6; i++) {
        taskMaximumVelocityHolder.push_back(taskLimits.maximumVelocity[i]);
        taskMaximumAccelerationHolder.push_back(taskLimits.maximumAcceleration[i]);
        TaskKpHolder.push_back(parametersPID.KpTask[i]);
        TaskKiHolder.push_back(parametersPID.KiTask[i]);
        TaskKdHolder.push_back(parametersPID.KdTask[i]);
    }
    etherCATArmJSON_["Task"] = {
        {"MaximumVelocity", taskMaximumVelocityHolder},
        {"MaximumAcceleration", taskMaximumAccelerationHolder},
        {"Kp", TaskKpHolder},
        {"Ki", TaskKiHolder},
        {"Kd", TaskKdHolder}
    };
}

}  // namespace crf::actuators::timarm
