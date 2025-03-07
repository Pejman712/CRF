/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Álvaro García González CERN BE/CEM/MRO 2023
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

#include "VisionUtility/PointCloud/Subsample.hpp"
#include "RGBDCameras/RealSenseCamera/RealSenseCamera.hpp"

using crf::sensors::cameras::Profile;

namespace po = boost::program_options;
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

    crf::sensors::rgbdcameras::RealSenseCamera cam(vm["serial_number"].as<std::string>());
    if (!cam.initialize()) {
        std::cout << "Failed to initialize the camera" << std::endl;
        return -1;
    }
    std::cout << "Real Sense started correctly\n" << std::endl;

    std::cout << "Setting resolutions.." << std::endl;;
    cam.setProfile(Profile(cv::Size(1920, 1080), 15));
    cam.setDepthProfile(Profile(cv::Size(1280, 720), 15));
    std::cout << "Resolutions set" << std::endl;;

    pcl::visualization::PCLVisualizer depthViewer("Depth Viewer");
    depthViewer.setBackgroundColor(0.0, 0.0, 0.0);
    depthViewer.addCoordinateSystem(0.25, "Origin");

    while (true) {
        // Read a new frame from the camera
        auto start = std::chrono::high_resolution_clock::now();

        // Call the function, here sort()
        auto frame = cam.capturePointCloud();

        // Get ending timepoint
        auto stop = std::chrono::high_resolution_clock::now();

        std::vector<float> sideLength = {0.05, 0.05, 0.05};
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloud =
            crf::utility::visionutility::pointcloud::subsample::voxelGridSubsample<pcl::PointXYZRGBA>(  // NOLINT
                frame.pointcloud, sideLength);

        std::cout << " PointCloud size: " << frame.pointcloud->size() << std::endl;
        std::cout << " Subsampling size: " << pointCloud->size() << std::endl;

        // Get duration.
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

        std::cout << "Time taken by RealSense Camera: "
            << duration.count() << " milliseconds" << std::endl;

        depthViewer.addPointCloud(pointCloud, "Point Cloud");
        depthViewer.spinOnce();
        depthViewer.removeAllPointClouds();
    }
    // Release the camera and close the window
    cv::destroyAllWindows();

    return 0;
}
