/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giancarlo D'Ago CERN BE/CEM/MRO 2023
 *
 * ====================================================================
*/

#include "MotionCapture/Vicon/Vicon.hpp"
#include "MotionCapture/MotionCaptureMarker.hpp"

namespace po = boost::program_options;

int main(int argc, char* argv[]) {
    po::options_description desc("Allowed options");
    desc.add_options()("help", "Show help message")
    ("host_name", po::value<std::string>(), "Host name")
    ("configuration", po::value<std::string>(), "Configuration file path");

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
    } catch ( std::exception& ) {
        std::cout << "Bad command line values" << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }
    po::notify(vm);

    if ( vm.count("help") ) {
        std::cout << desc << std::endl;
        return 0;
    }
    if ( !vm.count("host_name") ) {
        std::cout << "Missing host_name" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }
    if ( !vm.count("configuration") ) {
        std::cout << "Missing configuration" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    std::string HostName((vm["host_name"].as<std::string>()));
    std::ifstream Config(vm["configuration"].as<std::string>());
    nlohmann::json MotionCaptureJSON = nlohmann::json::parse(Config);

    crf::sensors::motioncapture::ViconMotionCapture Tracker(HostName, MotionCaptureJSON);
    Tracker.initialize();


    while ( true ) {
        crf::expected<std::vector<std::string>> objectList = Tracker.getObjectNames();
        std::cout << objectList.get_response() << std::endl;
        std::cout << (objectList.value()).size() << std::endl;
        std::cout << objectList.value()[0] << std::endl;

        for ( unsigned int i = 0 ; i < objectList.value().size() ; i++ ) {
            crf::expected<crf::utility::types::TaskPose> Pose =
                Tracker.getObjectPose(objectList.value()[i]);
            std::cout << objectList.value()[i] << ": [" << Pose.value() << " ]"
                << std::endl << std::endl;

            crf::expected<std::vector<crf::sensors::motioncapture::MotionCaptureMarker>>
                objectMarkerList = Tracker.getObjectMarkers(objectList.value()[i]);
            for (int j=0; j < objectMarkerList.value().size(); j++) {
                std::cout << objectMarkerList.value()[j].markerName << " ";
                std::cout << ": [" << objectMarkerList.value()[j].markerTranslation[0]
                    << " " << objectMarkerList.value()[j].markerTranslation[1]
                    << " " << objectMarkerList.value()[j].markerTranslation[2] << " ] ";
                std::cout << objectMarkerList.value()[j].markerOccluded << " ";
                std::cout << std::endl;
            }
            std::cout << std::endl;
        }

        std::cout << "------------------------------------------------------------" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return 1;
}
