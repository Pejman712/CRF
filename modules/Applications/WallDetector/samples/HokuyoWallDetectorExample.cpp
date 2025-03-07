/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <string>
#include <memory>
#include <vector>
#include <fstream>

#include <nlohmann/json.hpp>

#include "Types/TaskTypes/TaskPose.hpp"
#include "Laser/Hokuyo/HokuyoLaser.hpp"
#include "Laser/ILaser.hpp"
#include "RobotBase/RobotBaseConfiguration.hpp"
#include "WallDetector/IWallDetector.hpp"
#include "WallDetector/WallDetector.hpp"

int main(int argc, char* argv[]) {
    if (argc < 3) {
        printf("Too few arguments\n");
        printf("[1] Directory to robot configuration file (e.g. in respective config dir)\n");
        printf("[2] IP Adress laser\n");
        return 0;
    }
    std::string testAddress_ = __FILE__;
    testAddress_ =
        testAddress_.substr(0, testAddress_.find("samples/HokuyoWallDetectorExample.cpp"));
    auto robotConfig = std::make_shared<crf::robots::robotbase::RobotBaseConfiguration>();
    robotConfig->parse(argv[1]);

    nlohmann::json parameters;
    std::string localDir = testAddress_;
    localDir.append("config/WallDetectorParameters.json");
    std::ifstream config(localDir);
    config >> parameters;
    std::shared_ptr<crf::applications::walldetector::IWallDetector> distanceDetector;
    try {
        distanceDetector = std::make_shared<
            crf::applications::walldetector::WallDetector>(robotConfig, parameters);
    } catch (const std::exception& e) {
        std::cout << e.what();
        return 0;
    }
    auto laser = std::make_shared<crf::sensors::laser::HokuyoLaser>(
        crf::sensors::laser::HokuyoConnectionType::ETHERNET, argv[2], 10940);

    if (!laser->initialize()) {
        std::cout << "Could not initialize laser";
        return 0;
    }
    crf::utility::types::TaskPose laserPose({-0.333, .0, .0, .0, .0, .0});

    auto detectedParameters = distanceDetector->detectWall(laser->getPointCloud(), laserPose);
    if (detectedParameters.empty()) {
        std::cout << "No wall detected";
        return 0;
    } else {
        auto detectedWall = detectedParameters[0];
        std::cout << "theta " << detectedWall.theta << std::endl;
        std::cout << "length " << detectedWall.length << std::endl;
        std::cout << "distance " << detectedWall.distance << std::endl;
    }
    return 0;
}
