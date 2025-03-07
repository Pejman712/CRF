/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Álvaro García González CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include <string>
#include <iostream>
#include <memory>
#include <csignal>

#include <boost/program_options.hpp>

#include "Sockets/ISocket.hpp"
#include "Sockets/TCP/TCPSocket.hpp"
#include "Sockets/IPC/UnixSocket.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "RGBDCameras/RGBDCameraClient/RGBDCameraClient.hpp"

namespace po = boost::program_options;

using crf::communication::sockets::ISocket;
using crf::communication::sockets::TCPSocket;
using crf::communication::sockets::UnixSocket;
using crf::communication::datapacketsocket::PacketSocket;

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
        ("protocol", po::value<std::string>(), "Protocol type (available: tcp) [Required if port is set].") // NOLINT
        ("host", po::value<std::string>(), "IP of the server device")
        ("port", po::value<unsigned int>(), "Network port [1-65535] [Required if protocol is set].")
        ("ipc_name", po::value<std::string>(), "IPC socket name");

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
    if (!(vm.count("protocol") && vm.count("port") && vm.count("host")) && !vm.count("ipc_name")) {
        std::cout << "Missing at least one socket connection." << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    std::shared_ptr<ISocket> clientSocket;
    if (vm.count("port") && vm.count("host")) {
        clientSocket = std::make_shared<TCPSocket>(vm["host"].as<std::string>(),
            vm["port"].as<unsigned int>());
    } else if (vm.count("ipc_name")) {
        clientSocket = std::make_shared<UnixSocket>(vm["ipc_name"].as<std::string>());
    } else {
        std::cout << "Not able to create a connection with the given information" << std::endl;
        return 0;
    }
    std::shared_ptr<PacketSocket> socket(new PacketSocket(clientSocket));

    std::chrono::milliseconds serverReplyTimeout(50000);
    float streamerFrequency = 0;
    float priority = 1;
    crf::sensors::rgbdcameras::RGBDCameraClient cam(
        socket,
        serverReplyTimeout,
        streamerFrequency,
        priority);

    cam.initialize();
    // Create a window to display the camera feed
    cv::namedWindow("Camera Feed", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Depth Feed", cv::WINDOW_AUTOSIZE);
    std::vector<crf::sensors::cameras::Profile> profiles = cam.listProfiles();

    crf::expected<crf::sensors::cameras::Profile> depthProfile = cam.getDepthProfile();

    if (!depthProfile) {
        std::cout << "Not able to receive the current depth profile" << std::endl;
        return -1;
    }

    std::signal(SIGINT, signal_handler);
    while (gSignalStatus != SIGINT) {
        // Read a new frame from the camera
        cv::rgbd::RgbdFrame frame = cam.captureImageAndDepth();

        // Check if the frame was successfully captured
        if (frame.image.empty()) {
            std::cout << "No color image, end of video stream" << std::endl;
            break;
        }

        if (frame.depth.empty()) {
            std::cout << "No depth image, end of video stream" << std::endl;
            break;
        }
        // Display the frame in the window
        cv::imshow("Camera Feed", frame.image);
        cv::imshow("Depth Feed", frame.depth);

        // Press ESC to exit the program
        if (cv::waitKey(1) == 27) {
            break;
        }
    }

    std::cout << "Finishing the program" << std::endl;

    // Release the camera and close the window
    cv::destroyAllWindows();

    return 0;
}
