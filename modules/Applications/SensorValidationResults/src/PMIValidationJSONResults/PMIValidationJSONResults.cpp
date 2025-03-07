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

#include "SensorValidationResults/PMIValidationJSONResults/PMIValidationJSONResults.hpp"
#include "MailSender/CurlMailSender.hpp"
#include "MailSender/Mail.hpp"

namespace crf {
namespace applications {
namespace sensorvalidationresults {

PMIValidationJSONResults::PMIValidationJSONResults(const std::string &pathToCSV,
    const std::string &pathResultsFolder,
    const std::chrono::seconds &saveFileTimeInterval) :
    pathResultsFolder_(pathResultsFolder),
    saveFileTimeInterval_(saveFileTimeInterval),
    logger_("PMIValidationJSONResults"),
    validationResults_(),
    modifiedSensors_(),
    stopFileSaver_(false),
    resultsUpdated_(false),
    lastFileName_() {
    logger_->debug("Ctor");
    parseCSV(pathToCSV);
    fileSaverThread_ = std::thread(&PMIValidationJSONResults::fileSaver, this);
}

PMIValidationJSONResults::~PMIValidationJSONResults() {
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

bool PMIValidationJSONResults::setValues(const std::string &name, uint32_t priority,
    const nlohmann::json &json) {
    if (json.contains("expertName")) {
        expertName(name, priority, json["expertName"]);
    }
    if (json.contains("sector")) {
        sector(name, priority, json["sector"]);
    }
    if (json.contains("expectedDCUM")) {
        expectedDCUM(name, priority, json["expectedDCUM"].get<float>());
    }
    if (json.contains("measuredDCUM")) {
        measuredDCUM(name, priority, json["measuredDCUM"].get<float>());
    }
    if (json.contains("labelID")) {
        labelID(name, priority, json["labelID"]);
    }
    if (json.contains("labelQRCode")) {
        labelQRCode(name, priority, json["labelQRCode"]);
    }
    if (json.contains("isLabelCorrect")) {
        isLabelCorrect(name, priority, json["isLabelCorrect"].get<bool>());
    }
    if (json.contains("irradiationTime")) {
        irradiationTime(name, priority, json["irradiationTime"].get<float>());
    }
    if (json.contains("timeTestStart")) {
        timeTestStart(name, priority, json["timeTestStart"].get<std::time_t>());
    }
    if (json.contains("timeTestEnd")) {
        timeTestEnd(name, priority, json["timeTestEnd"].get<std::time_t>());
    }
    return true;
}

nlohmann::json PMIValidationJSONResults::getValues(const std::string &name) {
    return validationResults_[name];
}

bool PMIValidationJSONResults::contains(const std::string &name)  {
    return validationResults_.contains(name);
}

bool PMIValidationJSONResults::expertName(const std::string &name, uint32_t priority,
    const std::string &expertName) {
    return setStringValue("expertName", name, priority, expertName);
}

std::pair<uint32_t, std::string> PMIValidationJSONResults::expertName(
    const std::string &name) {
    return getStringValue("expertName", name);
}

bool PMIValidationJSONResults::sector(const std::string &name, uint32_t priority,
    const std::string &sector) {
    return setStringValue("sector", name, priority, sector);
}

std::pair<uint32_t, std::string> PMIValidationJSONResults::sector(
    const std::string &name) {
    return getStringValue("sector", name);
}

bool PMIValidationJSONResults::expectedDCUM(const std::string &name, uint32_t priority,
    float expectedDCUM) {
    return setFloatValue("expectedDCUM", name, priority, expectedDCUM);
}

std::pair<uint32_t, float> PMIValidationJSONResults::expectedDCUM(
    const std::string &name) {
    return getFloatValue("expectedDCUM", name);
}

bool PMIValidationJSONResults::measuredDCUM(const std::string &name, uint32_t priority,
    float measuredDCUM) {
    return setFloatValue("measuredDCUM", name, priority, measuredDCUM);
}

std::pair<uint32_t, float> PMIValidationJSONResults::measuredDCUM(
    const std::string &name) {
    return getFloatValue("measuredDCUM", name);
}

bool PMIValidationJSONResults::labelID(const std::string &name, uint32_t priority,
    const std::string &labelID) {
    return setStringValue("labelID", name, priority, labelID);
}

std::pair<uint32_t, std::string> PMIValidationJSONResults::labelID(
    const std::string &name) {
    return getStringValue("labelID", name);
}

bool PMIValidationJSONResults::labelQRCode(const std::string &name, uint32_t priority,
    const std::string &labelQRCode) {
    return setStringValue("labelQRCode", name, priority, labelQRCode);
}

std::pair<uint32_t, std::string> PMIValidationJSONResults::labelQRCode(
    const std::string &name) {
    return getStringValue("labelQRCode", name);
}

bool PMIValidationJSONResults::isLabelCorrect(const std::string &name, uint32_t priority,
    bool isLabelCorrect) {
    return setBooleanValue("labelCorrect", name, priority, isLabelCorrect);
}

std::pair<uint32_t, bool> PMIValidationJSONResults::isLabelCorrect(
    const std::string &name) {
    return getBooleanValue("labelCorrect", name);
}

bool PMIValidationJSONResults::irradiationTime(const std::string &name, uint32_t priority,
    float irradiationTime) {
    return setFloatValue("irradiationTime", name, priority, irradiationTime);
}

std::pair<uint32_t, float> PMIValidationJSONResults::irradiationTime(
    const std::string &name) {
    return getFloatValue("irradiationTime", name);
}

bool PMIValidationJSONResults::timeTestStart(const std::string &name, uint32_t priority,
    std::time_t time) {
    return setFloatValue("timeTestStart", name, priority, time);
}

std::pair<uint32_t, std::time_t> PMIValidationJSONResults::timeTestStart(
    const std::string &name) {
    return getFloatValue("timeTestStart", name);
}

bool PMIValidationJSONResults::timeTestEnd(const std::string &name, uint32_t priority,
    std::time_t time) {
    return setFloatValue("timeTestEnd", name, priority, time);
}

std::pair<uint32_t, std::time_t> PMIValidationJSONResults::timeTestEnd(
    const std::string &name) {
    return getFloatValue("timeTestEnd", name);
}

bool PMIValidationJSONResults::parseCSV(const std::string pathToCSV) {
    std::fstream file(pathToCSV);
    if (!file.is_open()) {
        logger_->critical("Faile to open {} ", pathToCSV);
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
        validationResults_[row[1]] = {
            {"expertName",      {{"Value", row[2]}, {"writerPriority", priority}}},
            {"sector",          {{"Value", row[0]}, {"writerPriority", priority}}},
            {"expectedDCUM",    {{"Value", std::stof(row[3])}, {"writerPriority", priority}}},
            {"measuredDCUM",    {{"Value", 0},      {"writerPriority", priority}}},
            {"labelID",         {{"Value", ""},     {"writerPriority", priority}}},
            {"labelQRCode",     {{"Value", ""},     {"writerPriority", priority}}},
            {"labelCorrect",    {{"Value", false},  {"writerPriority", priority}}},
            {"irradiationTime", {{"Value", std::stof(row[4])}, {"writerPriority", priority}}},
            {"timeTestStart",   {{"Value", 0},      {"writerPriority", priority}}},
            {"timeTestEnd",     {{"Value", 0},      {"writerPriority", priority}}}
        };
    }
    return true;
}

void PMIValidationJSONResults::fileSaver() {
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

void PMIValidationJSONResults::saveJSON(const nlohmann::json &input) {
    std::string newFileName;

    char hostname[HOST_NAME_MAX];
    gethostname(hostname, HOST_NAME_MAX);
    newFileName += hostname;

    std::time_t now = std::time(NULL);
    std::tm* ptm = std::localtime(&now);
    char buffer[32];
    std::strftime(buffer, 32, "_%d-%m-%Y_%H-%M-%S", ptm);

    newFileName = pathResultsFolder_ + hostname + "_PMI" + buffer + ".json";
    std::ofstream jsonFile(newFileName);

    logger_->info("Saving PMI validation results in the file {}", newFileName);
    jsonFile << input;
    std::remove(lastFileName_.c_str());
    lastFileName_ = newFileName;
}

nlohmann::json PMIValidationJSONResults::getJSONResults() {
    std::shared_lock lock(mutex_);
    nlohmann::json outputJSON;

    if (validationResults_.is_null()) {
        return outputJSON;
    }

    for (uint16_t i = 0; i < modifiedSensors_.size(); i++) {
        for (auto & element : validationResults_[modifiedSensors_[i]]) {
            if (element["writerPriority"] != std::numeric_limits<std::uint32_t>::max()) {
                // With only one we can add the sensor to the results and jump to the next one
                outputJSON[modifiedSensors_[i]] = validationResults_[modifiedSensors_[i]];
                break;
            }
        }
    }
    return outputJSON;
}

bool PMIValidationJSONResults::setStringValue(const std::string &parameter,
    const std::string &name, uint32_t priority, const std::string &value) {
    if (name.empty() || priority == 0 || value.empty()) {
        logger_->error("Parameters not valid");
        return false;
    }
    return setValue(parameter, name, priority, value);
}

bool PMIValidationJSONResults::setFloatValue(const std::string &parameter,
    const std::string &name, uint32_t priority, float value) {
    if (name.empty() || priority == 0 || value < 0) {
        logger_->error("Parameters not valid");
        return false;
    }
    return setValue(parameter, name, priority, value);
}

bool PMIValidationJSONResults::setBooleanValue(const std::string &parameter,
    const std::string &name, uint32_t priority, bool value) {
    if (name.empty() || priority == 0) {
        logger_->error("Parameters not valid");
        return false;
    }
    return setValue(parameter, name, priority, value);
}

std::pair<uint32_t, std::string> PMIValidationJSONResults::getStringValue(
    const std::string &parameter, const std::string &name) {
    std::shared_lock lock(mutex_);
    if (!validationResults_.contains(name)) {
        return std::pair<uint32_t, std::string>();
    }
    return std::pair<uint32_t, std::string>(
        validationResults_[name][parameter]["writerPriority"].get<uint32_t>(),
        validationResults_[name][parameter]["Value"].get<std::string>());
}

std::pair<uint32_t, float> PMIValidationJSONResults::getFloatValue(
    const std::string &parameter, const std::string &name) {
    std::shared_lock lock(mutex_);
    if (!validationResults_.contains(name)) {
        return std::pair<uint32_t, float>();
    }
    return std::pair<uint32_t, float>(
        validationResults_[name][parameter]["writerPriority"].get<uint32_t>(),
        validationResults_[name][parameter]["Value"].get<float>());
}

std::pair<uint32_t, bool> PMIValidationJSONResults::getBooleanValue(
    const std::string &parameter, const std::string &name) {
    std::shared_lock lock(mutex_);
    if (!validationResults_.contains(name)) {
        return std::pair<uint32_t, bool>();
    }
    return std::pair<uint32_t, bool>(
        validationResults_[name][parameter]["writerPriority"].get<uint32_t>(),
        validationResults_[name][parameter]["Value"].get<bool>());
}

}  // namespace sensorvalidationresults
}  // namespace applications
}  // namespace crf
