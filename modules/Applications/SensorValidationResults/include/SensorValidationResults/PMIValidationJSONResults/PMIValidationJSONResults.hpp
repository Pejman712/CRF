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

#include "SensorValidationResults/IPMIValidationResults.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace applications {
namespace sensorvalidationresults {

class PMIValidationJSONResults : public IPMIValidationResults {
 public:
    PMIValidationJSONResults(const std::string &pathToCSV,
        const std::string &pathResultsFolder,
        const std::chrono::seconds &saveFileTimeInterval);
    PMIValidationJSONResults(const PMIValidationJSONResults&) = delete;
    PMIValidationJSONResults(PMIValidationJSONResults&&) = delete;
    ~PMIValidationJSONResults();

    bool contains(const std::string &name) override;

    bool setValues(const std::string &monitorName, uint32_t priority,
        const nlohmann::json &json) override;
    nlohmann::json getValues(const std::string &monitorName) override;

    bool expertName(const std::string &name, uint32_t priority,
        const std::string &expertName) override;
    std::pair<uint32_t, std::string> expertName(const std::string &name) override;

    bool sector(const std::string &name, uint32_t priority,
        const std::string &sector) override;
    std::pair<uint32_t, std::string> sector(const std::string &name) override;

    bool expectedDCUM(const std::string &name, uint32_t priority,
        float expectedDCUM) override;
    std::pair<uint32_t, float> expectedDCUM(const std::string &name) override;

    bool measuredDCUM(const std::string &name, uint32_t priority,
        float measuredDCUM) override;
    std::pair<uint32_t, float> measuredDCUM(const std::string &name) override;

    bool labelID(const std::string &name, uint32_t priority,
        const std::string &labelID) override;
    std::pair<uint32_t, std::string> labelID(const std::string &name) override;

    bool labelQRCode(const std::string &name, uint32_t priority,
        const std::string &labelQRCode) override;
    std::pair<uint32_t, std::string> labelQRCode(const std::string &name) override;

    bool isLabelCorrect(const std::string &name, uint32_t priority,
        bool isLabelCorrect) override;
    std::pair<uint32_t, bool> isLabelCorrect(const std::string &name) override;

    bool irradiationTime(const std::string &name, uint32_t priority,
        float time) override;
    std::pair<uint32_t, float> irradiationTime(const std::string &name) override;

    bool timeTestStart(const std::string &name, uint32_t priority,
        std::time_t time) override;
    std::pair<uint32_t, std::time_t> timeTestStart(const std::string &name) override;

    bool timeTestEnd(const std::string &name, uint32_t priority,
        std::time_t time) override;
    std::pair<uint32_t, std::time_t> timeTestEnd(const std::string &name) override;

 private:
    std::string pathResultsFolder_;
    std::chrono::seconds saveFileTimeInterval_;
    crf::utility::logger::EventLogger logger_;
    nlohmann::json validationResults_;
    std::vector<std::string> modifiedSensors_;
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

    bool setStringValue(const std::string &parameter, const std::string &name, uint32_t priority,
        const std::string &value);
    bool setFloatValue(const std::string &parameter, const std::string &name, uint32_t priority,
        float value);
    bool setBooleanValue(const std::string &parameter, const std::string &name, uint32_t priority,
        bool value);

    template<typename T>
    bool setValue(const std::string &parameter, const std::string &name,
        uint32_t priority, const T &value) {
        std::unique_lock lock(mutex_);
        if (!validationResults_.contains(name)) {
            logger_->error("Unknown monitor name");
            return false;
        }
        modifiedSensors_.push_back(name);
        if (validationResults_[name][parameter]["writerPriority"] < priority) {
            logger_->warn("No access to write with given priority");
            return false;
        }
        validationResults_[name][parameter]["Value"] = value;
        validationResults_[name][parameter]["writerPriority"] = priority;
        resultsUpdated_ = true;
        return true;
    }

    std::pair<uint32_t, std::string> getStringValue(const std::string &parameter,
        const std::string &name);
    std::pair<uint32_t, float> getFloatValue(const std::string &parameter,
        const std::string &name);
    std::pair<uint32_t, bool> getBooleanValue(const std::string &parameter,
        const std::string &name);
};

}  // namespace sensorvalidationresults
}  // namespace applications
}  // namespace crf
