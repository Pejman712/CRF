/* © Copyright CERN 2021.  All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <string>
#include <ctime>
#include <vector>

#include "BLMTriggeringAnalysis/BLMTriggeringResult.hpp"

namespace crf {
namespace applications {
namespace blmtriggeringanalysis {

BLMTriggeringResult::BLMTriggeringResult() :
    monitorName_(),
    interactionPoint_(),
    timeBackgroundStart_(),
    timeBackgroundEnd_(),
    backgroundLevel_(),
    timeTestStart_(),
    timeTestEnd_(),
    signalMedian_(),
    signalMean_(),
    signalStandardDeviation_(),
    isSignalAboveThreshold_(false),
    blmsAboveThreshold_() {
}

BLMTriggeringResult::BLMTriggeringResult(const BLMTriggeringResult& input) {
    monitorName_ = input.monitorName();
    interactionPoint_ = input.interactionPoint();
    timeBackgroundStart_ = input.timeBackgroundStart();
    timeBackgroundEnd_ = input.timeBackgroundEnd();
    backgroundLevel_ = input.backgroundLevel();
    timeTestStart_ = input.timeTestStart();
    timeTestEnd_ = input.timeTestEnd();
    signalMedian_ = input.signalMedian();
    signalMean_ = input.signalMean();
    signalStandardDeviation_ = input.signalStandardDeviation();
    isSignalAboveThreshold_ = input.isSignalAboveThreshold();
    blmsAboveThreshold_ = input.blmsAboveThreshold();
}

std::string BLMTriggeringResult::monitorName() const {
    return monitorName_;
}

void BLMTriggeringResult::monitorName(const std::string &name) {
    monitorName_ = name;
    return;
}

std::string BLMTriggeringResult::interactionPoint() const {
    return interactionPoint_;
}

void BLMTriggeringResult::interactionPoint(const std::string &ip) {
    interactionPoint_ = ip;
    return;
}

std::time_t BLMTriggeringResult::timeBackgroundStart() const {
    return timeBackgroundStart_;
}

void BLMTriggeringResult::timeBackgroundStart(const std::time_t &timeValue) {
    timeBackgroundStart_ = timeValue;
    return;
}

std::time_t BLMTriggeringResult::timeBackgroundEnd() const {
    return timeBackgroundEnd_;
}

void BLMTriggeringResult::timeBackgroundEnd(const std::time_t &timeValue) {
    timeBackgroundEnd_ = timeValue;
    return;
}

float BLMTriggeringResult::backgroundLevel() const {
    return backgroundLevel_;
}

void BLMTriggeringResult::backgroundLevel(const float &level) {
    backgroundLevel_ = level;
    return;
}

std::time_t BLMTriggeringResult::timeTestStart() const {
    return timeTestStart_;
}

void BLMTriggeringResult::timeTestStart(const std::time_t &timeValue) {
    timeTestStart_ = timeValue;
    return;
}

std::time_t BLMTriggeringResult::timeTestEnd() const {
    return timeTestEnd_;
}

void BLMTriggeringResult::timeTestEnd(const std::time_t &timeValue) {
    timeTestEnd_ = timeValue;
    return;
}
float BLMTriggeringResult::signalMedian() const {
    return signalMedian_;
}

void BLMTriggeringResult::signalMedian(const float &median) {
    signalMedian_ = median;
    return;
}

float BLMTriggeringResult::signalMean() const {
    return signalMean_;
}

void BLMTriggeringResult::signalMean(const float &mean) {
    signalMean_ = mean;
    return;
}

float BLMTriggeringResult::signalStandardDeviation() const {
    return signalStandardDeviation_;
}

void BLMTriggeringResult::signalStandardDeviation(const float &standardDeviation) {
    signalStandardDeviation_ = standardDeviation;
    return;
}

bool BLMTriggeringResult::isSignalAboveThreshold() const {
    return isSignalAboveThreshold_;
}

void BLMTriggeringResult::isSignalAboveThreshold(const bool &aboveThreshold) {
    isSignalAboveThreshold_ = aboveThreshold;
    return;
}

std::vector<std::string> BLMTriggeringResult::blmsAboveThreshold() const {
    return blmsAboveThreshold_;
}

void BLMTriggeringResult::blmsAboveThreshold(const std::vector<std::string> &blmNames) {
    blmsAboveThreshold_ = blmNames;
    return;
}

}  // namespace blmtriggeringanalysis
}  // namespace applications
}  // namespace crf
