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

bool startVelThread = false;
bool startPosThread = false;

void printCavityPos(std::shared_ptr<EtherCATSRFCavityManager> cavityManager) {
    while (true) {
        if (startPosThread) {
            std::cout << "pos = " << rad2deg(cavityManager->getPosition().value()) <<
                " [deg]" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

void printCavityVel(std::shared_ptr<EtherCATSRFCavityManager> cavityManager) {
    while (true) {
        if (startVelThread) {
            std::cout << "vel = " << rad2deg(cavityManager->getVelocity().value()) <<
                " [deg/sec]" << std::endl;
            std::cout << "isTurning = " << cavityManager->isTurning().value() << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

int main(int argc, char* argv[]) {
    crf::utility::logger::EventLogger logger_("testCavityMotorInVel");

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

    std::thread thprintCavityPos(printCavityPos, cavityManager);
    if (thprintCavityPos.joinable()) {
        thprintCavityPos.detach();
    } else {
        logger_->error("thprintCavityPos not created and because it is not joinable");
        return false;
    }

    std::thread thprintCavityVel(printCavityVel, cavityManager);
    if (thprintCavityVel.joinable()) {
        thprintCavityVel.detach();
    } else {
        logger_->error("thprintCavityVel not created and because it is not joinable");
        return false;
    }

    logger_->info("cav pos = {} [deg]", cavityManager->getPosition());
    std::string targetVelocityStr = "";
    double targetVelocity = 0;
    crf::expected<double> pos = 0;
    bool stopCav = false;

    while (true) {
        std::cout << "Target vel [deg]?" << std::endl;
        std::cin >> targetVelocityStr;

        pos = cavityManager->getPosition();
        logger_->info("currentPos = {} [deg]", rad2deg(pos.value()));

        targetVelocity = deg2rad(std::stof(targetVelocityStr));

        startVelThread = true;
        startPosThread = false;
        if (cavityManager->setVelocity(targetVelocity)) {
            std::cout << "Stop the cavity ?" << std::endl;
            std::cin >> stopCav;
            if (stopCav) {
                if (!cavityManager->stop()) {
                    logger_->error("Impossible to stop the cavity.");
                }
            }
            pos = cavityManager->getPosition();
            logger_->info("currentPos = {} [deg]", rad2deg(pos.value()));
        }
        std::this_thread::sleep_for(std::chrono::seconds(3));
        startVelThread = false;
        startPosThread = false;
    }
    return 0;
}
