/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Camarero Vera CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <chrono>
#include <csignal>
#include <fstream>
#include <memory>
#include <string>

#include <nlohmann/json.hpp>

#include "CommUtility/CommunicationPacket.hpp"
#include "EventLogger/EventLogger.hpp"
#include "NetworkClient/FetchWriteClient.hpp"
#include "ObjectDetection/ObjectDetector.hpp"
#include "TIMAlignment/TIMAlignment.hpp"
#include "ZMQ/Request.hpp"

namespace {
    volatile std::sig_atomic_t gSignalStatus;
    void signal_handler(int signal) {
            gSignalStatus = signal;
    }
}   // unnamed namespace

using crf::vision::objectDetection::ObjectDetector;
using crf::communication::networkclient::FetchWriteClient;
using crf::applications::timalignment::TIMAlignment;
using crf::communication::zmqcomm::Request;

using namespace std::chrono_literals;   // NOLINT

int main(int argc, char** argv) {
    crf::utility::logger::EventLogger logger("TIM Alignment sample");
    if (argc < 2) {
        logger->error(R"(Needed argument no detected:
            1) Configuration file (Ej: ./modules/Applications/TIMAlignment/config/configFile.json)
            )");
        return -1;
    }

    std::signal(SIGTSTP, signal_handler);

    std::string configFile(argv[1]);
    std::ifstream config(configFile);
    nlohmann::json jConfig;
    config >> jConfig;

    auto detectorConfig = jConfig.at("Object_Detector");
    std::string cameraServerAddress = detectorConfig.at("Camera_Server_Address").get<std::string>();
    std::string inferenceServerAddress =
        detectorConfig.at("Inference_Server_Address").get<std::string>();
    float scale = detectorConfig.at("Scale").get<float>();
    auto clientConfig = jConfig.at("Fetch_Write_Client");
    std::string plcSocketAddress = clientConfig.at("PCL_Socket_Address").get<std::string>();
    int port = clientConfig.at("Port").get<int>();
    auto packetConfig = jConfig.at("Packet");
    Packets::FetchWritePacket packet;
    packet.dataBlockNumber_ = packetConfig.at("Data_Block_Number").get<int>();
    packet.startAddress_ = packetConfig.at("Start_Address").get<int>();
    packet.dataLength_ = packetConfig.at("End_Address").get<int>();

    auto cameraRequester = std::make_shared<Request>(cameraServerAddress);
    auto inferenceRequester = std::make_shared<Request>(inferenceServerAddress);

    auto detector = std::make_shared<ObjectDetector>(inferenceRequester, 5s, scale);
    logger->debug("PLC address: {0}, port: {1}", plcSocketAddress, port);
    auto plcComm = std::make_shared<FetchWriteClient>(plcSocketAddress, port);
    auto align = std::make_unique<TIMAlignment>(plcComm, detector, cameraRequester, packet);
    align->initialize();
    while (gSignalStatus != SIGTSTP) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return align->deinitialize();
}

