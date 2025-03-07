/* © Copyright CERN 2021.  All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <string>
#include <mutex>
#include <shared_mutex>
#include <limits.h>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <chrono>

#include "SensorValidationResults/BLMValidationJSONResults/BLMValidationJSONResults.hpp"
#include "BLMTriggeringAnalysis/BLMTriggeringResult.hpp"
#include "MailSender/CurlMailSender.hpp"
#include "MailSender/Mail.hpp"

namespace crf {
namespace applications {
namespace sensorvalidationresults {

BLMValidationJSONResults::BLMValidationJSONResults(const std::string &pathToCSV,
    const std::string &pathResultsFolder,
    const std::chrono::seconds &saveFileTimeInterval) :
    pathResultsFolder_(pathResultsFolder),
    saveFileTimeInterval_(saveFileTimeInterval),
    logger_("BLMValidationJSONResults"),
    validationResults_(),
    modifiedMonitors_(),
    stopFileSaver_(false),
    resultsUpdated_(false),
    lastFileName_() {
    logger_->debug("Ctor");
    parseCSV(pathToCSV);
    fileSaverThread_ = std::thread(&BLMValidationJSONResults::fileSaver, this);
}

BLMValidationJSONResults::~BLMValidationJSONResults() {
    logger_->debug("Dtor");

    stopFileSaver_ = true;
    if (fileSaverThread_.joinable()) {
        fileSaverThread_.join();
    }

    nlohmann::json results = getJSONResults();
    if (results.empty()) {
        return;
    }
    saveJSON(results);

    crf::communication::mailsender::MailAddress sender(senderMail);
    crf::communication::mailsender::MailAddress receiver(receiverMail);

    crf::communication::mailsender::Mail mail(sender);
    mail.addReceiver(receiver);
    mail.setSubject("BLM Commissioning Results");
    mail.setContent(results.dump());

    crf::communication::mailsender::CurlMailSender mailsender;
    mailsender.initialize();
    mailsender.send(mail);
}

bool BLMValidationJSONResults::contains(const std::string &monitorName)  {
    return validationResults_.contains(monitorName);
}

bool BLMValidationJSONResults::setValues(const std::string &monitorName, uint32_t priority,
    const nlohmann::json &json) {
    if (json.contains("expertMonitorName")) {
        expertMonitorName(monitorName, priority, json["expertMonitorName"]);
    }
    if (json.contains("interactionPoint")) {
        interactionPoint(monitorName, priority, json["interactionPoint"]);
    }
    if (json.contains("expectedDCUM")) {
        expectedDCUM(monitorName, priority, json["expectedDCUM"].get<float>());
    }
    if (json.contains("measuredDCUM")) {
        measuredDCUM(monitorName, priority, json["measuredDCUM"].get<float>());
    }
    if (json.contains("reachabilityLevel")) {
        reachabilityLevel(monitorName, priority, json["reachabilityLevel"]);
    }
    if (json.contains("location")) {
        location(monitorName, priority, json["location"]);
    }
    if (json.contains("position")) {
        position(monitorName, priority, json["position"]);
    }
    if (json.contains("labelID")) {
        labelID(monitorName, priority, json["labelID"]);
    }
    if (json.contains("labelQRCode")) {
        labelQRCode(monitorName, priority, json["labelQRCode"]);
    }
    if (json.contains("isLabelCorrect")) {
        isLabelCorrect(monitorName, priority, json["isLabelCorrect"].get<bool>());
    }
    if (json.contains("selectedRunningSum")) {
        selectedRunningSum(monitorName, priority, json["selectedRunningSum"]);
    }
    if (json.contains("irradiationTime")) {
        irradiationTime(monitorName, priority, json["irradiationTime"].get<float>());
    }
    if (json.contains("analysisThreshold")) {
        analysisThreshold(monitorName, priority, json["analysisThreshold"].get<float>());
    }
    if (json.contains("distanceToSource")) {
        distanceToSource(monitorName, priority, json["distanceToSource"].get<float>());
    }
    if (!json.contains("triggeringAnalysis")) {
        return true;
    }
    try {
        nlohmann::json analysisJSON = json["triggeringAnalysis"];
        crf::applications::blmtriggeringanalysis::BLMTriggeringResult analysis;
        analysis.timeBackgroundStart(analysisJSON["timeBackgroundStart"].get<std::time_t>());
        analysis.timeBackgroundEnd(analysisJSON["timeBackgroundEnd"].get<std::time_t>());
        analysis.backgroundLevel(analysisJSON["backgroundLevel"].get<float>());
        analysis.timeTestStart(analysisJSON["timeTestStart"].get<std::time_t>());
        analysis.timeTestEnd(analysisJSON["timeTestEnd"].get<std::time_t>());
        analysis.isSignalAboveThreshold(analysisJSON["isSignalAboveThreshold"].get<bool>());
        analysis.blmsAboveThreshold(analysisJSON["BLMsAboveThreshold"]);
        if (analysisJSON.contains("signalMedian")) {
            analysis.signalMedian(analysisJSON["signalMedian"].get<float>());
        }
        if (analysisJSON.contains("signalMean")) {
            analysis.signalMedian(analysisJSON["signalMean"].get<float>());
        }
        if (analysisJSON.contains("signalStandardDeviation")) {
            analysis.signalMedian(analysisJSON["signalStandardDeviation"].get<float>());
        }
        triggeringAnalysis(monitorName, priority, analysis);
    } catch (const std::exception& e) {
        logger_->error("Exception catched: {}", e.what());
        return false;
    }
    return true;
}

nlohmann::json BLMValidationJSONResults::getValues(const std::string &monitorName) {
    return validationResults_[monitorName];
}

bool BLMValidationJSONResults::expertMonitorName(const std::string &monitorName, uint32_t priority,
    const std::string &expertMonitorName) {
    return setStringValue("expertMonitorName", monitorName, priority, expertMonitorName);
}

std::pair<uint32_t, std::string> BLMValidationJSONResults::expertMonitorName(
    const std::string &monitorName) {
    return getStringValue("expertMonitorName", monitorName);
}

bool BLMValidationJSONResults::interactionPoint(const std::string &monitorName, uint32_t priority,
    const std::string &ip) {
    return setStringValue("interactionPoint", monitorName, priority, ip);
}

std::pair<uint32_t, std::string> BLMValidationJSONResults::interactionPoint(
    const std::string &monitorName) {
    return getStringValue("interactionPoint", monitorName);
}

bool BLMValidationJSONResults::expectedDCUM(const std::string &monitorName, uint32_t priority,
    float expectedDCUM) {
    return setFloatValue("expectedDCUM", monitorName, priority, expectedDCUM);
}

std::pair<uint32_t, float> BLMValidationJSONResults::expectedDCUM(
    const std::string &monitorName) {
    return getFloatValue("expectedDCUM", monitorName);
}

bool BLMValidationJSONResults::measuredDCUM(const std::string &monitorName, uint32_t priority,
    float measuredDCUM) {
    return setFloatValue("measuredDCUM", monitorName, priority, measuredDCUM);
}

std::pair<uint32_t, float> BLMValidationJSONResults::measuredDCUM(
    const std::string &monitorName) {
    return getFloatValue("measuredDCUM", monitorName);
}

bool BLMValidationJSONResults::reachabilityLevel(const std::string &monitorName, uint32_t priority,
    const std::string &reachibility) {
    return setStringValue("reachabilityLevel", monitorName, priority, reachibility);
}

std::pair<uint32_t, std::string> BLMValidationJSONResults::reachabilityLevel(
    const std::string &monitorName) {
    return getStringValue("reachabilityLevel", monitorName);
}

bool BLMValidationJSONResults::location(const std::string &monitorName, uint32_t priority,
    const std::string &location) {
    return setStringValue("location", monitorName, priority, location);
}

std::pair<uint32_t, std::string> BLMValidationJSONResults::location(
    const std::string &monitorName) {
    return getStringValue("location", monitorName);
}

bool BLMValidationJSONResults::position(const std::string &monitorName, uint32_t priority,
    const std::string &position) {
    return setStringValue("position", monitorName, priority, position);
}

std::pair<uint32_t, std::string> BLMValidationJSONResults::position(
    const std::string &monitorName) {
    return getStringValue("position", monitorName);
}

bool BLMValidationJSONResults::labelID(const std::string &monitorName, uint32_t priority,
    const std::string &labelID) {
    return setStringValue("labelID", monitorName, priority, labelID);
}

std::pair<uint32_t, std::string> BLMValidationJSONResults::labelID(
    const std::string &monitorName) {
    return getStringValue("labelID", monitorName);
}

bool BLMValidationJSONResults::labelQRCode(const std::string &monitorName, uint32_t priority,
    const std::string &labelQRCode) {
    return setStringValue("labelQRCode", monitorName, priority, labelQRCode);
}

std::pair<uint32_t, std::string> BLMValidationJSONResults::labelQRCode(
    const std::string &monitorName) {
    return getStringValue("labelQRCode", monitorName);
}

bool BLMValidationJSONResults::isLabelCorrect(const std::string &monitorName, uint32_t priority,
    bool isLabelCorrect) {
    return setBooleanValue("labelCorrect", monitorName, priority, isLabelCorrect);
}

std::pair<uint32_t, bool> BLMValidationJSONResults::isLabelCorrect(
    const std::string &monitorName) {
    return getBooleanValue("labelCorrect", monitorName);
}

bool BLMValidationJSONResults::selectedRunningSum(const std::string &monitorName, uint32_t priority,
    const std::string &runningSum) {
    return setStringValue("selectedRunningSum", monitorName, priority, runningSum);
}

std::pair<uint32_t, std::string> BLMValidationJSONResults::selectedRunningSum(
    const std::string &monitorName) {
    return getStringValue("selectedRunningSum", monitorName);
}

bool BLMValidationJSONResults::irradiationTime(const std::string &monitorName, uint32_t priority,
    float irradiationTime) {
    return setFloatValue("irradiationTime", monitorName, priority, irradiationTime);
}

std::pair<uint32_t, float> BLMValidationJSONResults::irradiationTime(
    const std::string &monitorName) {
    return getFloatValue("irradiationTime", monitorName);
}

bool BLMValidationJSONResults::analysisThreshold(const std::string &monitorName, uint32_t priority,
    float threshold) {
    return setFloatValue("analysisThreshold", monitorName, priority, threshold);
}

std::pair<uint32_t, float> BLMValidationJSONResults::analysisThreshold(
    const std::string &monitorName) {
    return getFloatValue("analysisThreshold", monitorName);
}

bool BLMValidationJSONResults::distanceToSource(const std::string &monitorName, uint32_t priority,
    float distance) {
    return setFloatValue("distanceToSource", monitorName, priority, distance);
}

std::pair<uint32_t, float> BLMValidationJSONResults::distanceToSource(
    const std::string &monitorName) {
    return getFloatValue("distanceToSource", monitorName);
}

bool BLMValidationJSONResults::triggeringAnalysis(const std::string &monitorName, uint32_t priority,
    blmtriggeringanalysis::BLMTriggeringResult triggeringAnalysis) {
    if (monitorName.empty() || priority == 0) {
        logger_->error("Parameters not valid");
        return false;
    }
    std::unique_lock lock(mutex_);
    if (!validationResults_.contains(monitorName)) {
        logger_->error("Unknown monitor name");
        return false;
    }
    modifiedMonitors_.push_back(monitorName);
    if (validationResults_[monitorName]["triggeringAnalysis"]["writerPriority"] < priority) {
        logger_->warn("No access to write with given priority");
        return false;
    }
    nlohmann::json &analysisJSON = validationResults_[monitorName]["triggeringAnalysis"];
    analysisJSON["timeBackgroundStart"] = triggeringAnalysis.timeBackgroundStart();
    analysisJSON["timeBackgroundEnd"] = triggeringAnalysis.timeBackgroundEnd();
    analysisJSON["backgroundLevel"] = triggeringAnalysis.backgroundLevel();
    analysisJSON["timeTestStart"] = triggeringAnalysis.timeTestStart();
    analysisJSON["timeTestEnd"] = triggeringAnalysis.timeTestEnd();
    analysisJSON["signalMedian"] = triggeringAnalysis.signalMedian();
    analysisJSON["signalMean"] = triggeringAnalysis.signalMean();
    analysisJSON["signalStandardDeviation"] = triggeringAnalysis.signalStandardDeviation();
    analysisJSON["signalAboveThreshold"] = triggeringAnalysis.isSignalAboveThreshold();
    analysisJSON["blmsAboveThreshold"] = triggeringAnalysis.blmsAboveThreshold();
    analysisJSON["writerPriority"] = priority;
    resultsUpdated_ = true;
    return true;
}

std::pair<uint32_t, blmtriggeringanalysis::BLMTriggeringResult> BLMValidationJSONResults::triggeringAnalysis(  // NOLINT
    const std::string &monitorName) {
    std::shared_lock lock(mutex_);
    if (!validationResults_.contains(monitorName)) {
        return std::pair<uint32_t, blmtriggeringanalysis::BLMTriggeringResult>();
    }
    blmtriggeringanalysis::BLMTriggeringResult output;
    nlohmann::json analysisJSON = validationResults_[monitorName]["triggeringAnalysis"];
    output.timeBackgroundStart(analysisJSON["timeBackgroundStart"].get<float>());
    output.timeBackgroundEnd(analysisJSON["timeBackgroundEnd"].get<float>());
    output.backgroundLevel(analysisJSON["backgroundLevel"].get<float>());
    output.timeTestStart(analysisJSON["timeTestStart"].get<float>());
    output.timeTestEnd(analysisJSON["timeTestEnd"].get<float>());
    output.signalMedian(analysisJSON["signalMedian"].get<float>());
    output.signalMean(analysisJSON["signalMean"].get<float>());
    output.signalStandardDeviation(analysisJSON["signalStandardDeviation"].get<float>());
    output.isSignalAboveThreshold(analysisJSON["signalAboveThreshold"].get<bool>());
    output.blmsAboveThreshold(analysisJSON["blmsAboveThreshold"]);
    return std::pair<uint32_t, blmtriggeringanalysis::BLMTriggeringResult >(
        analysisJSON["writerPriority"], output);
}

bool BLMValidationJSONResults::parseCSV(const std::string pathToCSV) {
    std::fstream file(pathToCSV);
    if (!file.is_open()) {
        logger_->critical("Fail to open {} ", pathToCSV);
        return false;
    }
    std::vector<std::string> row;
    std::string line, cell;
    int lineNumber = 0;
    uint32_t priority = std::numeric_limits<std::uint32_t>::max();
    while (file >> line) {
        if (lineNumber == 0) {
            lineNumber = 1;
            continue;
        }

        row.clear();
        std::stringstream s(line);
        while (std::getline(s, cell, ',')) {
            row.push_back(cell);
        }

        validationResults_[row[0]] = {
            {"expertMonitorName",  {{"Value", row[1]},            {"writerPriority", priority}}},
            {"interactionPoint",   {{"Value", row[3]},            {"writerPriority", priority}}},
            {"expectedDCUM",       {{"Value", std::stof(row[2])}, {"writerPriority", priority}}},
            {"measuredDCUM",       {{"Value", 0},                 {"writerPriority", priority}}},
            {"reachabilityLevel",  {{"Value", row[9]},            {"writerPriority", priority}}},
            {"location",           {{"Value", row[10]},           {"writerPriority", priority}}},
            {"position",           {{"Value", row[11]},           {"writerPriority", priority}}},
            {"labelID",            {{"Value", ""},                {"writerPriority", priority}}},
            {"labelQRCode",        {{"Value", ""},                {"writerPriority", priority}}},
            {"labelCorrect",       {{"Value", false},             {"writerPriority", priority}}},
            {"selectedRunningSum", {{"Value", row[5]},            {"writerPriority", priority}}},
            {"irradiationTime",    {{"Value", std::stof(row[6])}, {"writerPriority", priority}}},
            {"analysisThreshold",  {{"Value", std::stof(row[8])}, {"writerPriority", priority}}},
            {"distanceToSource",   {{"Value", std::stof(row[7])}, {"writerPriority", priority}}},
            {"triggeringAnalysis", {
                {"timeBackgroundStart", 0},
                {"timeBackgroundEnd", 0},
                {"backgroundLevel", 0},
                {"timeTestStart", 0},
                {"timeTestEnd", 0},
                {"signalMedian", 0},
                {"signalMean", 0},
                {"signalStandardDeviation", 0},
                {"signalAboveThreshold", false},
                {"blmsAboveThreshold", {""}},
                {"writerPriority", priority}
            }}
        };
    }
    return true;
}

void BLMValidationJSONResults::fileSaver() {
    auto currentTime = std::chrono::high_resolution_clock::now();
    auto lastSavedFileTime = std::chrono::high_resolution_clock::now();
    while (!stopFileSaver_) {
        currentTime = std::chrono::high_resolution_clock::now();
        auto timeSinceLastSavedFile = std::chrono::duration_cast<std::chrono::milliseconds>(
            currentTime-lastSavedFileTime);
        if (resultsUpdated_ && timeSinceLastSavedFile > saveFileTimeInterval_) {
            nlohmann::json results = getJSONResults();
            if (!results.empty()) {
                saveJSON(results);
            }
            resultsUpdated_ = false;
            lastSavedFileTime = std::chrono::high_resolution_clock::now();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
}

void BLMValidationJSONResults::saveJSON(const nlohmann::json &input) {
    std::string newFileName;

    char hostname[HOST_NAME_MAX];
    gethostname(hostname, HOST_NAME_MAX);
    newFileName += hostname;

    std::time_t now = std::time(NULL);
    std::tm* ptm = std::localtime(&now);
    char buffer[32];
    std::strftime(buffer, 32, "_%d-%m-%Y_%H-%M-%S", ptm);

    newFileName = pathResultsFolder_ + hostname + "_BLM" + buffer + ".json";
    std::ofstream jsonFile(newFileName);

    logger_->info("Saving BLM validation results in the file {}", newFileName);
    jsonFile << input;
    std::remove(lastFileName_.c_str());
    lastFileName_ = newFileName;
}

nlohmann::json BLMValidationJSONResults::getJSONResults() {
    std::shared_lock lock(mutex_);
    nlohmann::json outputJSON;

    if (validationResults_.is_null()) {
        return outputJSON;
    }

    for (uint16_t i = 0; i < modifiedMonitors_.size(); i++) {
        for (auto & element : validationResults_[modifiedMonitors_[i]]) {
            if (element["writerPriority"] != std::numeric_limits<std::uint32_t>::max()) {
                // With only one we can add the sensor to the results and jump to the next one
                outputJSON[modifiedMonitors_[i]] = validationResults_[modifiedMonitors_[i]];
                break;
            }
        }
    }
    return outputJSON;
}

bool BLMValidationJSONResults::setStringValue(const std::string &parameter,
    const std::string &monitorName, uint32_t priority, const std::string &value) {
    if (monitorName.empty() || priority == 0 || value == "") {
        logger_->error("Parameters not valid: monitorName {}, priority {}, value {}",
            monitorName, priority, value);
        return false;
    }
    return setValue(parameter, monitorName, priority, value);
}

bool BLMValidationJSONResults::setFloatValue(const std::string &parameter,
    const std::string &monitorName, uint32_t priority, float value) {
    if (monitorName.empty() || priority == 0 || value < 0) {
        logger_->error("Parameters not valid");
        return false;
    }
    return setValue(parameter, monitorName, priority, value);
}

bool BLMValidationJSONResults::setBooleanValue(const std::string &parameter,
    const std::string &monitorName, uint32_t priority, bool value) {
    if (monitorName.empty() || priority == 0) {
        logger_->error("Parameters not valid");
        return false;
    }
    return setValue(parameter, monitorName, priority, value);
}

std::pair<uint32_t, std::string> BLMValidationJSONResults::getStringValue(
    const std::string &parameter, const std::string &monitorName) {
    std::shared_lock lock(mutex_);
    if (!validationResults_.contains(monitorName)) {
        return std::pair<uint32_t, std::string>();
    }
    return std::pair<uint32_t, std::string>(
        validationResults_[monitorName][parameter]["writerPriority"].get<uint32_t>(),
        validationResults_[monitorName][parameter]["Value"].get<std::string>());
}

std::pair<uint32_t, float> BLMValidationJSONResults::getFloatValue(
    const std::string &parameter, const std::string &monitorName) {
    std::shared_lock lock(mutex_);
    if (!validationResults_.contains(monitorName)) {
        return std::pair<uint32_t, float>();
    }
    return std::pair<uint32_t, float>(
        validationResults_[monitorName][parameter]["writerPriority"].get<uint32_t>(),
        validationResults_[monitorName][parameter]["Value"].get<float>());
}

std::pair<uint32_t, bool> BLMValidationJSONResults::getBooleanValue(
    const std::string &parameter, const std::string &monitorName) {
    std::shared_lock lock(mutex_);
    if (!validationResults_.contains(monitorName)) {
        return std::pair<uint32_t, bool>();
    }
    return std::pair<uint32_t, bool>(
        validationResults_[monitorName][parameter]["writerPriority"].get<uint32_t>(),
        validationResults_[monitorName][parameter]["Value"].get<bool>());
}

}  // namespace sensorvalidationresults
}  // namespace applications
}  // namespace crf
