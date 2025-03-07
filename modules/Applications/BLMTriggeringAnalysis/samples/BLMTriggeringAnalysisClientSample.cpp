/* © Copyright CERN 2021.  All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <string>
#include <iostream>
#include <memory>

#include <boost/program_options.hpp>

#include "Sockets/ISocket.hpp"
#include "Sockets/TCP/TCPSocket.hpp"
#include "Sockets/IPC/UnixSocket.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "BLMTriggeringAnalysis/BLMTriggeringAnalysisClient/BLMTriggeringAnalysisClient.hpp"

namespace po = boost::program_options;

using crf::communication::sockets::ISocket;
using crf::communication::sockets::TCPSocket;
using crf::communication::sockets::UnixSocket;
using crf::communication::datapacketsocket::PacketSocket;
using crf::applications::blmtriggeringanalysis::BLMTriggeringAnalysisClient;
using crf::applications::blmtriggeringanalysis::BLMTriggeringResult;

int main(int argc, char* argv[]) {
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Show help message")
        ("blm_name", po::value<std::string>(), "BLM name (eg. BLMQI.C1R1).")
        ("ip", po::value<std::string>(), "Interaction Point of the BLM (eg. LHC_IP1).")
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
    if (!vm.count("ip")) {
        std::cout << "Missing at interaction point." << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }
    if (!(vm.count("protocol") && vm.count("port") && vm.count("host")) && !vm.count("ipc_name")) {
        std::cout << "Missing at least one socket connection." << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    std::shared_ptr<ISocket> client;
    if (vm.count("port") && vm.count("host")) {
        client = std::make_shared<TCPSocket>(vm["host"].as<std::string>(),
            vm["port"].as<unsigned int>());
    } else if (vm.count("ipc_name")) {
        client = std::make_shared<UnixSocket>(vm["ipc_name"].as<std::string>());
    } else {
        std::cout << "Not able to create a connection with the given information" << std::endl;
        return 0;
    }
    std::shared_ptr<PacketSocket> socket(new PacketSocket(client));

    std::shared_ptr<BLMTriggeringAnalysisClient> triggerinAnalysis(new BLMTriggeringAnalysisClient(
        socket, std::chrono::milliseconds(120000)));

    if (!triggerinAnalysis->calculateBackgroundLevels(vm["ip"].as<std::string>())) {
        std::cout << "ERROR: Not able to calculate the background" << std::endl;
        return -1;
    }
    std::cout << "Background level calculated." << std::endl;


    std::vector<BLMTriggeringResult> allBLM =
        triggerinAnalysis->executeInteractionPointTriggeringAnalysis(vm["ip"].as<std::string>());

    std::cout << "Number of BLM analyzed: " << allBLM.size() << std::endl;

    for (BLMTriggeringResult singleBLM : allBLM) {
        std::cout << std::endl;
        std::cout << "----------------------BLM----------------------" << std::endl;
        std::cout << "monitorName: " << singleBLM.monitorName() << std::endl;
        std::cout << "signalMedian: " << singleBLM.signalMedian() << std::endl;
        std::cout << "signalMean: " << singleBLM.signalMean() << std::endl;
        std::time_t start = singleBLM.timeBackgroundStart();
        std::cout << "timeBackgroundStart: " << std::ctime(&start);
        std::time_t end = singleBLM.timeBackgroundEnd();
        std::cout << "timeBackgroundEnd: " << std::ctime(&end);
        std::cout << "isSignalAboveThreshold: " << singleBLM.isSignalAboveThreshold() << std::endl;
    }


    BLMTriggeringResult singleBLM = triggerinAnalysis->executeSpecificTriggeringAnalysis(
        vm["blm_name"].as<std::string>());

    std::cout << std::endl;
    std::cout << "----------------------BLM----------------------" << std::endl;
    std::cout << "monitorName: " << singleBLM.monitorName() << std::endl;
    std::cout << "signalMedian: " << singleBLM.signalMedian() << std::endl;
    std::cout << "signalMean: " << singleBLM.signalMean() << std::endl;
    std::time_t start = singleBLM.timeBackgroundStart();
    std::cout << "timeBackgroundStart: " << std::ctime(&start);
    std::time_t end = singleBLM.timeBackgroundEnd();
    std::cout << "timeBackgroundEnd: " << std::ctime(&end);
    std::cout << "isSignalAboveThreshold: " << singleBLM.isSignalAboveThreshold() << std::endl;
    for (auto iter : singleBLM.blmsAboveThreshold()) {
        std::cout << "blmsAboveThreshold: " << iter << std::endl;
    }

    float value = triggerinAnalysis->getBLMValueWithoutBackground(vm["blm_name"].as<std::string>());
    std::cout << "BLM Value: " << value << std::endl;

    return 0;
}
