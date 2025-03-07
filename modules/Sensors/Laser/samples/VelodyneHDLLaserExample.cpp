/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Prados Sesmero CERN EN/SMM/MRO 
 *
 *  ==================================================================================================
 */

#include <iostream>
#include <string>
#include <memory>
#include <chrono>
#include <csignal>
#include <stdlib.h>

#include <nlohmann/json.hpp>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

#include "EventLogger/EventLogger.hpp"
#include "Laser/VelodyneHDL/VelodyneHDLLaser.hpp"

void signalHandler(int signal) {
    std::cout << "Too much time waiting for a new point cloud" << std::endl;
    exit(signal);
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cout << "Too few arguments : [1] CONFIG_FILENAME " << std::endl;
        return -1;
    }

    std::ifstream laserData(argv[1]);
    nlohmann::json laserJSON = nlohmann::json::parse(laserData);
    std::unique_ptr<crf::sensors::laser::VelodyneHDLLaser> velodyne {
        new crf::sensors::laser::VelodyneHDLLaser {laserJSON}};

    std::signal(SIGTERM, signalHandler);

    if (!velodyne->initialize()) {
        std::cout << "Not possible to initialize the velodyne" << std::endl;
        return -1;
    }
    std::cout << "Sensor initialized" << std::endl;

    pcl::visualization::CloudViewer viewer("Viewer");
    char hola;
    while (!viewer.wasStopped()) {
        auto start = std::chrono::system_clock::now();
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud = velodyne->getPointCloud();
        std::chrono::duration<float, std::milli> time = std::chrono::system_clock::now() - start;
        std::cout << "PointCloud adquisition time: " << time.count() << std::endl;

        if (cloud == nullptr) {
            std::cout << "Cloud empty" << std::endl;
            break;
        }

        viewer.showCloud(cloud);
        std::this_thread::sleep_for(std::chrono::milliseconds(rand() % 2000));  // NOLINT
    }

    return 0;
}
