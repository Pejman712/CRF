/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Adrien Luthi CERN EN/SMM/MRO 2023
 *
 *  ===============================================================================================
 */

#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <chrono>
#include <inttypes.h>
#include <fstream>

#include "EtherCATDevices/EtherCATManager.hpp"
#include "EtherCATDevices/IEtherCATMotor.hpp"
#include "EtherCATDevices/EtherCATMotor.hpp"
#include "EtherCATDevices/EtherCATDef.hpp"

#include "SRFCavityManager/EtherCATSRFCavityManager/EtherCATSRFCavityManager.hpp"

using crf::devices::ethercatdevices::EtherCATManager;
using crf::devices::ethercatdevices::IEtherCATMotor;
using crf::devices::ethercatdevices::EtherCATMotor;
using crf::actuators::srfcavityManager::EtherCATSRFCavityManager;
using crf::devices::ethercatdevices::modesofoperation::ProfileVelocityMode;
using crf::devices::ethercatdevices::modesofoperation::ProfilePositionMode;

double deg2rad(double degree) {
    return (degree * (M_PI / 180));
}

double rad2deg(double radian) {
    return (radian * (180 / M_PI));
}

int main(int argc, char* argv[]) {
    crf::utility::logger::EventLogger logger_("testCavityMotorSwitchingMOp");

    double pos = 0;
    float target = 0;
    float expected = 0;
    bool answer = false;

    std::string configPath = __FILE__;
    std::cout << configPath << std::endl;
    configPath = configPath.substr(0, configPath.find("samples"));
    std::cout << configPath << std::endl;
    configPath += "config/SRFCavityConfig.json";
    std::cout << configPath << std::endl;

    std::ifstream robotData(configPath);
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);

    std::shared_ptr<EtherCATSRFCavityManager> cavityManager;

    //////////////// ECManager & Cavity motor initialization ////////////////
    std::shared_ptr<EtherCATManager> ECManager_;
    std::shared_ptr<IEtherCATMotor> cavityMotor_;
    std::string cavityMotorPortName_ = "enx00051bd72e05";
    int nECSlaves_ = 1;
    int ECIOMapSize_ = 4096;
    const int ECcycleTime_ = 1000;

    ECManager_ = std::make_shared<EtherCATManager>
        (cavityMotorPortName_, nECSlaves_, ECIOMapSize_, ECcycleTime_);
    std::vector<std::shared_ptr<IEtherCATMotor>> tempVect;
    tempVect.push_back(std::make_shared<EtherCATMotor>(1, ECManager_));
    cavityMotor_ = tempVect[0];
    if (!ECManager_->initialize()) {
        logger_->error("Can't initialize Manager");
        throw;
    }
    logger_->info("Manager initialized");
    if (!cavityMotor_->initialize()) {
        logger_->error("Cannot initialize Motor");
        throw;
    }
    if (!ECManager_->configureIOMap()) {
        logger_->error("Can't configure EtherCAT IOMap");
        throw;
    }
    if (!cavityMotor_->bindPDOs()) {
        logger_->error("Can't bind PDO for Motor");
        throw;
    }
    if (!ECManager_->enterOp()) {
        logger_->error("Manager can't enter in operation");
        throw;
    }
    cavityManager = std::make_shared<EtherCATSRFCavityManager>(robotJSON, cavityMotor_);
    //////////////////////////////////////////////////////////////////////////

    if (!cavityManager->initialize()) {
        logger_->error("Impossible to initialize");
        return false;
    }
    if (!cavityManager->enableMotor()) {
        logger_->error("Impossible to enableMotor");
        return false;
    }

    cavityManager->setPosition(deg2rad(90), true);

    std::this_thread::sleep_for(std::chrono::seconds(1));

    cavityManager->setVelocity(deg2rad(5));

    std::this_thread::sleep_for(std::chrono::seconds(5));

    cavityManager->stop();

    std::this_thread::sleep_for(std::chrono::seconds(1));

    cavityManager->setPosition(deg2rad(150), true);

    std::this_thread::sleep_for(std::chrono::seconds(1));

    cavityManager->setVelocity(deg2rad(-5));

    std::this_thread::sleep_for(std::chrono::seconds(5));

    cavityManager->stop();

    return 0;
}
