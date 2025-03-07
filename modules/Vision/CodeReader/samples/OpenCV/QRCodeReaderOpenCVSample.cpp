/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <array>
#include <chrono>
#include <iostream>
#include <atomic>
#include <csignal>

#include <boost/optional.hpp>
#include <opencv2/opencv.hpp>

#include "Sockets/TCP/TCPSocket.hpp"
#include "Cameras/CameraClient/CameraClient.hpp"
#include "CodeReader/Detector/QRCodeCVDetector/QRCodeCVDetector.hpp"
#include "CodeReader/Decoder/QRCodeCVDecoder/QRCodeCVDecoder.hpp"
#include "CodeReader/Decoder/QRCodeCVDecoder/QRCodeCVCurvedDecoder.hpp"
#include "CodeReader/Reader/CodeBackgroundReader/CodeBackgroundReader.hpp"

namespace {
    volatile std::sig_atomic_t gSignalStatus;
    void signal_handler(int signal) {
        std::cout << "Caught signal Ctrl-Z" << signal << std::endl;
        gSignalStatus = signal;
    }
}  // unnamed namespace

int main(int argc, char *argv[]) {
    std::signal(SIGTSTP, signal_handler);

    std::chrono::milliseconds serverReplyTimeout(25000);
    auto clientSocketCamera = std::make_shared<crf::communication::sockets::TCPSocket>(
        "localhost", 8001);
    auto socketCamera = std::make_shared<crf::communication::datapacketsocket::PacketSocket>(
        clientSocketCamera);
    auto camera = std::make_shared<crf::sensors::cameras::CameraClient>(
        socketCamera, serverReplyTimeout, 1, 1);
    if (!camera->initialize()) {
        std::cout << "Failed to initialize the camera" << std::endl;
        return -1;
    }
    std::cout << "Camera started correctly" << std::endl;

    auto detector = std::make_shared<crf::vision::codereader::QRCodeCVDetector>();
    auto decoder = std::make_shared<crf::vision::codereader::QRCodeCVCurvedDecoder>();
    auto QRReader = std::make_shared<crf::vision::codereader::CodeBackgroundReader>(
        camera, decoder, detector);

    // Physical size of the QR
    double width = 0.04;  // meters
    double height = 0.04;  // meters

    // Let the thread start and get ready
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    QRReader->start();

    while (gSignalStatus != SIGTSTP) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if (QRReader->isDetected()) {
            std::optional<std::string> result = QRReader->getCode();
            if (!result) continue;
            std::cout << "QR detected: " << result.value() << std::endl;
            break;
        } else {
            std::cout << "..." << std::endl;
        }
    }
    std::cout << "Received closing command" << std::endl;
    return 0;
}
