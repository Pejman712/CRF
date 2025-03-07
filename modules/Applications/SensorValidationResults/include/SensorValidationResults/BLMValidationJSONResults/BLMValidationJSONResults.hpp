/* © Copyright CERN 2021.  All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>
#include <utility>
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>

#include "SensorValidationResults/IBLMValidationResults.hpp"
#include "BLMTriggeringAnalysis/BLMTriggeringResult.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace applications {
namespace sensorvalidationresults {

class BLMValidationJSONResults : public IBLMValidationResults {
 public:
    BLMValidationJSONResults(const std::string &pathToCSV,
        const std::string &pathResultsFolder,
        const std::chrono::seconds &saveFileTimeInterval);
    BLMValidationJSONResults(const BLMValidationJSONResults&) = delete;
    BLMValidationJSONResults(BLMValidationJSONResults&&) = delete;
    ~BLMValidationJSONResults();

    bool contains(const std::string &monitorName) override;

    bool setValues(const std::string &monitorName, uint32_t priority,
        const nlohmann::json &json) override;
    nlohmann::json getValues(const std::string &monitorName) override;

    bool expertMonitorName(const std::string &monitorName, uint32_t priority,
        const std::string &expertMonitorName) override;
    std::pair<uint32_t, std::string> expertMonitorName(const std::string &monitorName) override;

    bool interactionPoint(const std::string &monitorName, uint32_t priority,
        const std::string &ip) override;
    std::pair<uint32_t, std::string> interactionPoint(const std::string &monitorName) override;

    bool expectedDCUM(const std::string &monitorName, uint32_t priority,
        const float expectedDCUM) override;
    std::pair<uint32_t, float> expectedDCUM(const std::string &monitorName) override;

    bool measuredDCUM(const std::string &monitorName, uint32_t priority,
        float measuredDCUM) override;
    std::pair<uint32_t, float> measuredDCUM(const std::string &monitorName) override;

    bool reachabilityLevel(const std::string &monitorName, uint32_t priority,
        const std::string &reachibility) override;
    std::pair<uint32_t, std::string> reachabilityLevel(const std::string &monitorName) override;

    bool location(const std::string &monitorName, uint32_t priority,
        const std::string &location) override;
    std::pair<uint32_t, std::string> location(const std::string &monitorName) override;

    bool position(const std::string &monitorName, uint32_t priority,
        const std::string &position) override;
    std::pair<uint32_t, std::string> position(const std::string &monitorName) override;

    bool labelID(const std::string &monitorName, uint32_t priority,
        const std::string &labelID) override;
    std::pair<uint32_t, std::string> labelID(const std::string &monitorName) override;

    bool labelQRCode(const std::string &monitorName, uint32_t priority,
        const std::string &labelQRCode) override;
    std::pair<uint32_t, std::string> labelQRCode(const std::string &monitorName) override;

    bool isLabelCorrect(const std::string &monitorName, uint32_t priority,
        bool isLabelCorrect) override;
    std::pair<uint32_t, bool> isLabelCorrect(const std::string &monitorName) override;

    bool selectedRunningSum(const std::string &monitorName, uint32_t priority,
        const std::string &runningSum) override;
    std::pair<uint32_t, std::string> selectedRunningSum(const std::string &monitorName) override;

    bool irradiationTime(const std::string &monitorName, uint32_t priority,
        float irradiationTime) override;
    std::pair<uint32_t, float> irradiationTime(const std::string &monitorName) override;

    bool analysisThreshold(const std::string &monitorName, uint32_t priority,
        float threshold) override;
    std::pair<uint32_t, float> analysisThreshold(const std::string &monitorName) override;

    bool distanceToSource(const std::string &monitorName, uint32_t priority,
        float distance) override;
    std::pair<uint32_t, float> distanceToSource(const std::string &monitorName) override;

    bool triggeringAnalysis(const std::string &monitorName, uint32_t priority,
        blmtriggeringanalysis::BLMTriggeringResult triggeringAnalysis) override;
    std::pair<uint32_t, blmtriggeringanalysis::BLMTriggeringResult> triggeringAnalysis(
        const std::string &monitorName) override;

 private:
    std::string pathResultsFolder_;
    std::chrono::seconds saveFileTimeInterval_;
    crf::utility::logger::EventLogger logger_;
    nlohmann::json validationResults_;
    std::vector<std::string> modifiedMonitors_;
    std::atomic<bool> stopFileSaver_;
    std::thread fileSaverThread_;
    std::shared_mutex mutex_;
    std::atomic<bool> resultsUpdated_;
    std::string lastFileName_;
    const std::string senderMail = "tim@cern.ch";
    const std::string receiverMail = "alejandro.diaz.rosales@cern.ch";

    bool parseCSV(const std::string pathToCSV);
    void fileSaver();
    void saveJSON(const nlohmann::json &input);
    nlohmann::json getJSONResults();

    bool setStringValue(const std::string &parameter, const std::string &monitorName,
        uint32_t priority, const std::string &value);
    bool setFloatValue(const std::string &parameter, const std::string &monitorName,
        uint32_t priority, float value);
    bool setBooleanValue(const std::string &parameter, const std::string &monitorName,
        uint32_t priority, bool value);

    template<typename T>
    bool setValue(const std::string &parameter, const std::string &monitorName,
        uint32_t priority, const T &value) {
        std::unique_lock lock(mutex_);
        if (!validationResults_.contains(monitorName)) {
            logger_->error("Unknown monitor name");
            return false;
        }
        modifiedMonitors_.push_back(monitorName);
        if (validationResults_[monitorName][parameter]["writerPriority"] < priority) {
            logger_->warn("No access to write with given priority");
            return false;
        }
        validationResults_[monitorName][parameter]["Value"] = value;
        validationResults_[monitorName][parameter]["writerPriority"] = priority;
        resultsUpdated_ = true;
        return true;
    }

    std::pair<uint32_t, std::string> getStringValue(const std::string &parameter,
        const std::string &monitorName);
    std::pair<uint32_t, float> getFloatValue(const std::string &parameter,
        const std::string &monitorName);
    std::pair<uint32_t, bool> getBooleanValue(const std::string &parameter,
        const std::string &monitorName);
};

}  // namespace sensorvalidationresults
}  // namespace applications
}  // namespace crf
