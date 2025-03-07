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
#include <mutex>
#include <chrono>
#include <exception>
#include <stdexcept>
#include <thread>

#include <nlohmann/json.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/hdl_grabber.h>
#include <pcl/console/parse.h>

#include "EventLogger/EventLogger.hpp"
#include "Laser/VelodyneHDL/VelodyneHDLLaserPCL.hpp"

void signalHandler(int signal) {
    std::cout << "Too much time waiting for a new point cloud" << std::endl;
    exit(signal);
}

int main(int argc, char** argv) {
    crf::utility::logger::EventLogger logger("VelodyneHDLLaserPCLExample");
    if (argc != 2) {
        logger->error("Too few arguments : [1] CONFIG_FILENAME ");
        return -1;
    }

    std::ifstream laserData(argv[1]);
    nlohmann::json laserJSON = nlohmann::json::parse(laserData);
    crf::sensors::laser::VelodyneHDLLaserPCL velodyne(laserJSON);

    std::signal(SIGTERM, signalHandler);

    if (!velodyne.initialize()) {
        logger->error("Unable to initialize the sensor");
        return -1;
    }

    logger->info("Sensor initialized");

    pcl::visualization::CloudViewer viewer("Viewer");
    while (!viewer.wasStopped()) {
        auto start = std::chrono::system_clock::now();
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud = velodyne.getPointCloud();
        std::chrono::duration<float, std::milli> time = std::chrono::system_clock::now() - start;
        std::cout << "PointCloud adquisition time: " << time.count() << std::endl;

        if (cloud == nullptr) {
            std::cout << "Cloud empty" << std::endl;
            return 0;
        }
        std::cout << "Size: " << cloud->height << " " << cloud->width << std::endl;
        viewer.showCloud(cloud);
        std::this_thread::sleep_for(std::chrono::milliseconds(rand() % 2000));  // NOLINT
    }

    return 0;
}
