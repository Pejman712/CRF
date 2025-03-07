/* © Copyright CERN 2021.  All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */


#include <string>
#include <vector>
#include <thread>
#include <condition_variable>

#include <nlohmann/json.hpp>

#include "BLMTriggeringAnalysis/BLMTriggeringAnalysisClient/BLMTriggeringAnalysisClient.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "BLMTriggeringAnalysis/BLMTriggeringResult.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"

namespace crf {
namespace applications {
namespace blmtriggeringanalysis {

BLMTriggeringAnalysisClient::BLMTriggeringAnalysisClient(
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
    const std::chrono::milliseconds& serverReplyTimeout) :
    socket_(socket),
    serverReplyTimeout_(serverReplyTimeout),
    logger_("BLMTriggeringAnalysisClient"),
    stopGrabberThread_(),
    grabberThread_(),
    socketMutex_(),
    calculateBackgroundLevelsMutex_(),
    calculateBackgroundLevelsResult_(),
    calculateBackgroundLevelsCV_(),
    executeInteractionPointTriggeringAnalysisMutex_(),
    interactionPointTriggeringAnalysisResult_(),
    executeInteractionPointTriggeringAnalysisCV_(),
    executeSpecificTriggeringAnalysisMutex_(),
    specificTriggeringAnalysisResult_(),
    executeSpecificTriggeringAnalysisCV_(),
    getBLMValueWithoutBackgroundMutex_(),
    blmValueWithoutBackgroundResult_(),
    getBLMValueWithoutBackgroundCV_() {
    logger_->debug("Ctor");
    if (!socket_->open()) {
        logger_->warn("Failed to connect to BLM Triggering Analysis server");
        throw std::runtime_error("Failed to connect to BLM Triggering Analysis server");
    }
    stopGrabberThread_ = false;
    grabberThread_ = std::thread(&BLMTriggeringAnalysisClient::grabber, this);
}

BLMTriggeringAnalysisClient::~BLMTriggeringAnalysisClient() {
    logger_->debug("Dtor");
    if (!socket_->close()) {
        logger_->warn("Failed to close socket");
    }
    stopGrabberThread_ = true;
    if (grabberThread_.joinable()) {
        grabberThread_.join();
    }
}

bool BLMTriggeringAnalysisClient::calculateBackgroundLevels(
    const std::string &ip) {
    logger_->debug("calculateBackgroundLevels");

    communication::datapackets::JSONPacket json;
    json.data["command"] = "calculateBackgroundLevels";
    json.data["interactionPoint"] = ip;
    std::unique_lock<std::mutex> socketLock(socketMutex_);
    socket_->write(json, json.getHeader());
    socketLock.unlock();

    std::unique_lock<std::mutex> lock(calculateBackgroundLevelsMutex_);
    if (calculateBackgroundLevelsCV_.wait_for(lock, serverReplyTimeout_) ==
        std::cv_status::timeout) {
        logger_->error("Timeout for server reply");
        return false;
    }
    return calculateBackgroundLevelsResult_;
}

std::vector<BLMTriggeringResult> BLMTriggeringAnalysisClient::executeInteractionPointTriggeringAnalysis( //NOLINT
    const std::string &ip) {
    logger_->debug("executeInteractionPointTriggeringAnalysis");

    communication::datapackets::JSONPacket json;
    json.data["command"] = "executeInteractionPointTriggeringAnalysis";
    json.data["interactionPoint"] = ip;
    std::unique_lock<std::mutex> socketLock(socketMutex_);
    socket_->write(json, json.getHeader());
    socketLock.unlock();

    std::unique_lock<std::mutex> lock(executeInteractionPointTriggeringAnalysisMutex_);
    if (executeInteractionPointTriggeringAnalysisCV_.wait_for(lock, serverReplyTimeout_) ==
        std::cv_status::timeout) {
        logger_->error("Timeout for server reply");
        return std::vector<BLMTriggeringResult>();
    }
    return interactionPointTriggeringAnalysisResult_;
}

BLMTriggeringResult BLMTriggeringAnalysisClient::executeSpecificTriggeringAnalysis(
    const std::string &blmName) {
    logger_->debug("executeSpecificTriggeringAnalysis");

    communication::datapackets::JSONPacket json;
    json.data["command"] = "executeSpecificTriggeringAnalysis";
    json.data["blmName"] = blmName;
    std::unique_lock<std::mutex> socketLock(socketMutex_);
    socket_->write(json, json.getHeader());
    socketLock.unlock();

    std::unique_lock<std::mutex> lock(executeSpecificTriggeringAnalysisMutex_);
    if (executeSpecificTriggeringAnalysisCV_.wait_for(lock, serverReplyTimeout_) ==
        std::cv_status::timeout) {
        logger_->error("Timeout for server reply");
        return BLMTriggeringResult();
    }
    return specificTriggeringAnalysisResult_;
}

float BLMTriggeringAnalysisClient::getBLMValueWithoutBackground(const std::string &blmName)  {
    logger_->debug("getBLMValueWithoutBackground");

    communication::datapackets::JSONPacket json;
    json.data["command"] = "getBLMValueWithoutBackground";
    json.data["blmName"] = blmName;
    std::unique_lock<std::mutex> socketLock(socketMutex_);
    socket_->write(json, json.getHeader());
    socketLock.unlock();

    std::unique_lock<std::mutex> lock(getBLMValueWithoutBackgroundMutex_);
    if (getBLMValueWithoutBackgroundCV_.wait_for(lock, serverReplyTimeout_) ==
        std::cv_status::timeout) {
        logger_->error("Timeout for server reply");
        return 0;
    }
    return blmValueWithoutBackgroundResult_;
}

void BLMTriggeringAnalysisClient::grabber() {
    while (!stopGrabberThread_) {
        if (!socket_->isOpen()) {
            return;
        }
        auto retreiveValue = socket_->read();
        if (!retreiveValue) {
            continue;
        }
        auto buffer = retreiveValue.value().first;
        auto header = retreiveValue.value().second;

        if (header.type() != communication::datapackets::JSON_PACKET_TYPE) {
            logger_->warn("Only JSON packets are expected from this communication point");
            continue;
        }
        communication::datapackets::JSONPacket json;
        if (!json.deserialize(buffer)) {
            logger_->warn("Failed to deserialize json packet: {}", buffer);
            continue;
        }

        try {
            if (json.data["command"].get<std::string>() != "reply") {
                logger_->warn("Unexpected JSON content");
                continue;
            }
            std::string jsonReplyCommand = json.data["replyCommand"].get<std::string>();
            if (jsonReplyCommand == "calculateBackgroundLevels") {
                std::unique_lock<std::mutex> lock(calculateBackgroundLevelsMutex_);
                calculateBackgroundLevelsResult_ = json.data["message"].get<bool>();
                calculateBackgroundLevelsCV_.notify_one();
                continue;
            } else if (jsonReplyCommand == "executeInteractionPointTriggeringAnalysis") {
                std::unique_lock<std::mutex> lock(executeInteractionPointTriggeringAnalysisMutex_);
                interactionPointTriggeringAnalysisResult_ = json2BLMTriggeringResultVector(
                    json.data["message"]);
                executeInteractionPointTriggeringAnalysisCV_.notify_one();
                continue;
            } else if (jsonReplyCommand == "executeSpecificTriggeringAnalysis") {
                std::unique_lock<std::mutex> lock(executeSpecificTriggeringAnalysisMutex_);
                specificTriggeringAnalysisResult_ = json2BLMTriggeringResult(json.data["message"]);
                executeSpecificTriggeringAnalysisCV_.notify_one();
                continue;
            } else if (jsonReplyCommand == "getBLMValueWithoutBackground") {
                std::unique_lock<std::mutex> lock(getBLMValueWithoutBackgroundMutex_);
                blmValueWithoutBackgroundResult_ = json.data["message"].get<float>();
                getBLMValueWithoutBackgroundCV_.notify_one();
                continue;
            } else if (jsonReplyCommand == "error") {
                logger_->warn("There was an error with the sent message: {}",
                    json.data["message"].get<std::string>());
                continue;
            } else {
                logger_->warn("Unknown message: {}", json.data["message"].get<std::string>());
                continue;
            }
        } catch (const std::exception& ex) {
            logger_->warn("Failed reading json response: {}", ex.what());
            continue;
        }
        logger_->warn("Unknonw JSON message");
        continue;
    }
}

BLMTriggeringResult BLMTriggeringAnalysisClient::json2BLMTriggeringResult(
    const nlohmann::json &input) {
    BLMTriggeringResult output;
    output.monitorName(input["monitorName"].get<std::string>());
    output.interactionPoint(input["interactionPoint"].get<std::string>());
    output.timeBackgroundStart(static_cast<std::time_t>(input["timeBackgroundStart"]));
    output.timeBackgroundEnd(static_cast<std::time_t>(input["timeBackgroundEnd"]));
    output.backgroundLevel(input["backgroundLevel"].get<float>());
    output.timeTestStart(static_cast<std::time_t>(input["timeTestStart"]));
    output.timeTestEnd(static_cast<std::time_t>(input["timeTestEnd"]));
    if (input.contains("signalMedian")) {
        output.signalMedian(input["signalMedian"].get<float>());
    }
    if (input.contains("signalMean")) {
        output.signalMean(input["signalMean"].get<float>());
    }
    if (input.contains("signalStandardDeviation")) {
        output.signalStandardDeviation(input["signalStandardDeviation"].get<float>());
    }
    if (input.contains("BLMsAboveThreshold")) {
        std::vector<std::string> blmNames;
        for (nlohmann::json name : input["BLMsAboveThreshold"]) {
            blmNames.push_back(name.get<std::string>());
        }
        output.blmsAboveThreshold(blmNames);
    }
    output.isSignalAboveThreshold(input["isSignalAboveThreshold"].get<int>());
    return output;
}

std::vector<BLMTriggeringResult> BLMTriggeringAnalysisClient::json2BLMTriggeringResultVector(
    const nlohmann::json &input) {
    std::vector<BLMTriggeringResult> output;
    for (nlohmann::json singleJSON : input) {
        output.push_back(json2BLMTriggeringResult(singleJSON));
    }
    return output;
}

}  // namespace blmtriggeringanalysis
}  // namespace applications
}  // namespace crf
