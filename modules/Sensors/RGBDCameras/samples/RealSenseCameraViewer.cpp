/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Prados Sesmero CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#include <csignal>
#include <iostream>
#include <memory>
#include <string>

#include <boost/program_options.hpp>
#include <opencv2/highgui.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include "RGBDCameras/RealSenseCamera/RealSenseCamera.hpp"

namespace po = boost::program_options;

namespace {
volatile std::sig_atomic_t gSignalStatus;
void signal_handler(int signal) {
    gSignalStatus = signal;
}
}  // unnamed namespace

int main(int argc, char* argv[]) {
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Show help message")
        ("serial_number", po::value<std::string>(),
            "Camera serial number (the serial number is directly written on the camera)");

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
    } catch (std::exception&) {
        std::cout << "Bad command line values" << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 0;
    }
    if (!vm.count("serial_number")) {
        std::cout << "Missing serial number" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    std::shared_ptr<crf::sensors::rgbdcameras::IRGBDCamera> camera =
        std::make_shared<crf::sensors::rgbdcameras::RealSenseCamera>(
            vm["serial_number"].as<std::string>());

    if (!camera->initialize()) {
        std::cout << "Failed to initialize the camera" << std::endl;
        return -1;
    }

    if (!camera->setProfile(camera->listProfiles()[0], camera->listDepthProfiles()[0])) {
        std::cout << "Failed to set profile in the camera" << std::endl;
        return -1;
    }

    std::cout << "Real Sense started correctly\n";

    pcl::visualization::PCLVisualizer depthViewer("Depth Viewer");
    depthViewer.setBackgroundColor(0.0, 0.0, 0.0);
    depthViewer.addCoordinateSystem(0.25, "Origin");

    std::signal(SIGINT, signal_handler);
    while (gSignalStatus != SIGINT) {
        auto frame = camera->capturePointCloud();
        if (frame.pointcloud == nullptr) continue;
        depthViewer.addPointCloud(frame.pointcloud, "Point Cloud");
        depthViewer.spinOnce(0);
        depthViewer.removeAllPointClouds();

        // cv::Mat frame = camera->captureImage();
        // if (frame.empty()) continue;
        // cv::imshow("Window", frame);
        // cv::waitKey(1);
    }

    camera->deinitialize();
    std::cout << "Received closing command" << std::endl;
    return 0;
}
