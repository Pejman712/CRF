
/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <csignal>
#include <cstdlib>
#include <memory>
#include <thread>
#include <fstream>
#include <string>
#include <vector>
#include <chrono>

#include "EventLogger/EventLogger.hpp"
#include "PersonFollower/PersonDetector.hpp"
#include "PersonFollower/PersonTracker.hpp"
#include "PersonFollower/PersonFollower.hpp"
#include "RobotBaseControllers/RobotBaseControllerFactory.hpp"
#include "ObjectDetection/ObjectDetectorPython.hpp"
#include "ZMQ/Request.hpp"
#include "Cameras/UvcCamera/UvcCamera.hpp"
#include "nlohmann/json.hpp"
#include "CANSocket/CANSocket.hpp"
#include "CHARMBot/CHARMBot.hpp"
#include "IPC/IPC.hpp"
#include "IPC/MMAP.hpp"
#include "RobotBase/RobotBaseConfiguration.hpp"
#include "Laser/Hokuyo/HokuyoLaser.hpp"
#include "WallDetector/WallDetector.hpp"

#define HOKUYO_DEFAULT_BAUDRATE 115200

using crf::applications::personfollower::PersonDetector;
using crf::applications::personfollower::PersonFollower;
using crf::applications::personfollower::PersonTracker;
using crf::communication::zmqcomm::Request;
using crf::vision::objectDetection::ObjectDetectorPython;
using crf::applications::robotbasecontroller::RobotBaseControllerFactory;
using crf::sensors::cameras::UvcCamera;
using crf::sensors::laser::HokuyoLaser;
using crf::sensors::laser::HokuyoConnectionType;
using crf::communication::lasercommunicationpoint::LaserCommunicationPoint;
using crf::applications::walldetector::WallDetector;

namespace {
volatile std::sig_atomic_t gSignalStatus;
void signal_handler(int signal) {
    gSignalStatus = signal;
}
}   // unnamed namespace

int main(int argc, char const *argv[]) {
    crf::utility::logger::EventLogger logger("PersonFollower Sample");
    if (argc < 2) {
        logger->error(R"(Configuration file necessary (e.g. ~/../PersonFollower/config/PersonDetectionConfig.json))
        )");
        return -1;
    }
    std::signal(SIGTSTP, signal_handler);

    std::string configFile(argv[1]);
    std::ifstream config(configFile);
    nlohmann::json jConfig;
    config >> jConfig;
    auto personDetectorConfig = jConfig.at("PersonDetectorConfig");
    std::string inferenceServerAddress =
      personDetectorConfig.at("inferenceServerAddress").get<std::string>();
    std::string uvcCameraConnectionPort =
      personDetectorConfig.at("uvcCameraConnectionPort").get<std::string>();
    std::string laserIP =
      personDetectorConfig.at("laserIP").get<std::string>();
    std::string laserCommPoint =
      personDetectorConfig.at("laserCommPoint").get<std::string>();
    std::string laserConfigFile =
      personDetectorConfig.at("laserConfigFile").get<std::string>();
    std::string robotConfigFile =
      personDetectorConfig.at("robotConfigFile").get<std::string>();
    std::string canInterface =
      personDetectorConfig.at("canInterface").get<std::string>();
    std::string wallParameters =
      personDetectorConfig.at("wallParameters").get<std::string>();
    // objDetect detector
    auto zmqRequest = std::make_shared<Request>(inferenceServerAddress);
    auto objectDetector = std::make_shared<ObjectDetectorPython>(zmqRequest);
    // UvcCamera
    auto camera = std::make_shared<UvcCamera>(uvcCameraConnectionPort);
    // laser comm point
    auto laser = std::make_shared<HokuyoLaser>(HokuyoConnectionType::ETHERNET, laserIP, 10940);
    std::shared_ptr<MMAP> publisher = MMAP::CreateWriterPtr(laserCommPoint, 2e6);
    std::shared_ptr<MMAP> receiver = MMAP::CreateReaderPtr(laserCommPoint, 2e6);
    std::shared_ptr<MMAP> receiver2 = MMAP::CreateReaderPtr(laserCommPoint, 2e6);
    receiver->setBlockingRead(std::chrono::seconds(1));
    receiver2->setBlockingRead(std::chrono::seconds(1));

    auto laserCommunicationPoint =
      std::make_shared<LaserCommunicationPoint>(laser, nullptr, publisher);
    laserCommunicationPoint->initialize();
    // personDetector
    auto personDetector = std::make_shared<PersonDetector>(camera, objectDetector);
    // personTracker
    std::ifstream lConfig(laserConfigFile);
    nlohmann::json laserConfig;
    lConfig >> laserConfig;
    std::string robotConfigFileDir(robotConfigFile);
    auto baseConfig = std::make_shared<crf::robots::robotbase::RobotBaseConfiguration>();
    baseConfig->parse(robotConfigFileDir);
    auto personTracker = std::make_shared<PersonTracker>(
      receiver, personDetector, laserConfig, baseConfig);
    // wallDetector
    std::ifstream l1Config(wallParameters);
    nlohmann::json wallParametersJson;
    l1Config >> wallParametersJson;
    auto wallDetector = std::make_shared<WallDetector>(baseConfig, wallParametersJson);
    // initializing robotBaseController
    auto socket = std::make_shared<CANSocket>(canInterface);
    auto bot = std::make_shared<crf::robots::robotbase::CHARMBot>(socket, robotConfigFileDir);
    if (!bot->initialize()) {
        logger->error("Could not initialize the robot");
        return -1;
    }
    auto robotBaseConfig = bot->getConfiguration();
    std::vector<bool> selectedDim(TASK_SPACE_SIZE);
    std::array<int, SE2_SPACE_SIZE> dimBase = {LINEAR_DIM_ID, TRANSVERSAL_DIM_ID, ROTATION_DIM_ID};
    for (unsigned int i = 0; i < dimBase.size(); i++) {
        selectedDim[dimBase[i]] = true;
    }
    // Select if velocity- or position- based control
    auto selectedControlMode = crf::applications::robotbasecontroller::ControlMode::Velocity;
    crf::applications::robotbasecontroller::RobotBaseControllerFactory factory(bot);
    auto controller = factory.create(selectedControlMode);
    // estimating cameraPose manually
    crf::utility::types::TaskPose cameraPose({0.1, 0.1, 0.2, 0, 0, 0});
    auto personFollower = std::make_unique<PersonFollower>(
      personTracker, controller, cameraPose, receiver2, wallDetector);
    if (!personFollower->initialize()) {
        logger->error("could not initialize PersonFollower");
        return -1;
    }
    while (gSignalStatus != SIGTSTP) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    if (!personFollower->deinitialize()) {
        logger->error("Failed to stop followPerson");
        return -1;
    }
    return 0;
}
