#pragma once
/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Camarero Vera CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include "CommonInterfaces/IInitializable.hpp"
#include "EventLogger/EventLogger.hpp"
#include "NetworkClient/FetchWritePacket.hpp"
#include "NetworkClient/INetworkClient.hpp"
#include "ObjectDetection/IObjectDetector.hpp"
#include "ZMQ/Request.hpp"

namespace crf {
namespace applications {
namespace timalignment {

class TIMAlignment : public utility::commoninterfaces::IInitializable {
 public:
    TIMAlignment(std::shared_ptr<communication::networkclient::INetworkClient> client,
                 std::shared_ptr<vision::objectDetection::IObjectDetector> objectDetector,
                 std::shared_ptr<communication::zmqcomm::Request> cameraRequester,
                 const Packets::FetchWritePacket& packet,
                 const std::chrono::milliseconds& timeOut = std::chrono::milliseconds(1000));
    ~TIMAlignment() = default;
    bool initialize() override;
    bool deinitialize() override;
    TIMAlignment(const TIMAlignment& other) = delete;
    TIMAlignment(TIMAlignment&& other) = delete;

 private:
    utility::logger::EventLogger logger_;
    std::shared_ptr<communication::networkclient::INetworkClient> client_;
    std::shared_ptr<vision::objectDetection::IObjectDetector> objectDetector_;
    std::shared_ptr<communication::zmqcomm::Request> cameraRequester_;
    Packets::FetchWritePacket packet_;
    std::chrono::milliseconds timeOut_;
    bool stopSignal_;

    std::thread app_;

    enum State {
        guides2,
        collimator1,
        guide1NoCollimator,
        noDeviceDetected
    };
    enum TIMMovement {
        IDLE        = 0,
        FORWARD     = 1,
        BACKWARD    = 2,
        STOP        = 3
    };
    struct Positions {
        int leftGuide;
        int rightGuide;
        int collimatorPosition;
        int mediumOfFrame;
    };

    const int MEDIUM_OF_REFERENCE_FRAME = 640 / 2;
    const float THRESHOLD = 15.0 / MEDIUM_OF_REFERENCE_FRAME;

    void run();
    int getCenter(int leftPosition, int rightPosition);
    boost::optional<cv::Mat> getFrameFromServer();
    std::pair<State, Positions> checkingDetection();
    void moveWith2Guides(Positions positions);
    void moveWith1Collimator(Positions positions);
    void moveWith1Guide0Collimator(Positions positions);
    void noDetection();
};

}  // namespace timalignment
}  // namespace applications
}  // namespace crf

