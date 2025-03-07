/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <condition_variable>

#include <nlohmann/json.hpp>

#include "BLMTriggeringAnalysis/IBLMTriggeringAnalysis.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "BLMTriggeringAnalysis/BLMTriggeringResult.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace applications {
namespace blmtriggeringanalysis {

class BLMTriggeringAnalysisClient : public IBLMTriggeringAnalysis {
 public:
    BLMTriggeringAnalysisClient(
        std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
        const std::chrono::milliseconds& serverReplyTimeout);
    BLMTriggeringAnalysisClient(const BLMTriggeringAnalysisClient&) = delete;
    BLMTriggeringAnalysisClient(BLMTriggeringAnalysisClient&&) = delete;
    ~BLMTriggeringAnalysisClient();

    bool calculateBackgroundLevels(const std::string &ip) override;
    std::vector<BLMTriggeringResult> executeInteractionPointTriggeringAnalysis(const std::string &ip) override; // NOLINT
    BLMTriggeringResult executeSpecificTriggeringAnalysis(const std::string &blmName) override;
    float getBLMValueWithoutBackground(const std::string &blmName) override;

 private:
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket_;
    const std::chrono::milliseconds serverReplyTimeout_;
    crf::utility::logger::EventLogger logger_;
    std::atomic<bool> stopGrabberThread_;
    std::thread grabberThread_;
    std::mutex socketMutex_;
    std::mutex calculateBackgroundLevelsMutex_;
    bool calculateBackgroundLevelsResult_;
    std::condition_variable calculateBackgroundLevelsCV_;
    std::mutex executeInteractionPointTriggeringAnalysisMutex_;
    std::vector<BLMTriggeringResult> interactionPointTriggeringAnalysisResult_;
    std::condition_variable executeInteractionPointTriggeringAnalysisCV_;
    std::mutex executeSpecificTriggeringAnalysisMutex_;
    BLMTriggeringResult specificTriggeringAnalysisResult_;
    std::condition_variable executeSpecificTriggeringAnalysisCV_;
    std::mutex getBLMValueWithoutBackgroundMutex_;
    float blmValueWithoutBackgroundResult_;
    std::condition_variable getBLMValueWithoutBackgroundCV_;

    void grabber();
    BLMTriggeringResult json2BLMTriggeringResult(const nlohmann::json &input);
    std::vector<BLMTriggeringResult> json2BLMTriggeringResultVector(const nlohmann::json &input);
};

}  // namespace blmtriggeringanalysis
}  // namespace applications
}  // namespace crf
