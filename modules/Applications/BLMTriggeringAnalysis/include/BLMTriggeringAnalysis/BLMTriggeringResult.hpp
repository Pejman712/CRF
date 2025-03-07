/* © Copyright CERN 2021.  All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>
#include <ctime>
#include <vector>

namespace crf {
namespace applications {
namespace blmtriggeringanalysis {

/*
 * @brief Class that works as container to save and access the results of triggering the Beam Loss
 *        Monitors (BLM).
 */
class BLMTriggeringResult {
 public:
    BLMTriggeringResult();
    BLMTriggeringResult(const BLMTriggeringResult&);
    ~BLMTriggeringResult() = default;

    /*
     * @brief Gets the name or identifier of the Beam Loss Monitor.
     * @return The name as a string.
     */
    std::string monitorName() const;
    /*
     * @brief Sets the name or identifier of the Beam Loss Monitor.
     * @param The name as a string.
     */
    void monitorName(const std::string &name);
    /*
     * @brief Gets the interaction point r of the Beam Loss Monitor.
     * @return The ip as a string.
     */
    std::string interactionPoint() const;
    /*
     * @brief Sets the interaction point of the Beam Loss Monitor.
     * @param The ip as a string.
     */
    void interactionPoint(const std::string &ip);
    /*
     * @brief Gets the start time when the background level is calculated.
     * @return Arithmetic type capable of representing times. Although not defined, this is almost
     *         always an integral value holding the number of seconds (not counting leap seconds)
     *         since 00:00, Jan 1 1970 UTC, corresponding to POSIX time.
     */
    std::time_t timeBackgroundStart() const;
    /*
     * @brief Sets the start time when the background level is calculated.
     * @param Arithmetic type capable of representing times. Although not defined, this is almost
     *        always an integral value holding the number of seconds (not counting leap seconds)
     *        since 00:00, Jan 1 1970 UTC, corresponding to POSIX time.
     */
    void timeBackgroundStart(const std::time_t &timeValue);
    /*
     * @brief Gets the end time when the background level is calculated.
     * @return Arithmetic type capable of representing times. Although not defined, this is almost
     *         always an integral value holding the number of seconds (not counting leap seconds)
     *         since 00:00, Jan 1 1970 UTC, corresponding to POSIX time.
     */
    std::time_t timeBackgroundEnd() const;
    /*
     * @brief Sets the end time when the background level is calculated.
     * @param Arithmetic type capable of representing times. Although not defined, this is almost
     *        always an integral value holding the number of seconds (not counting leap seconds)
     *        since 00:00, Jan 1 1970 UTC, corresponding to POSIX time.
     */
    void timeBackgroundEnd(const std::time_t &timeValue);
    /*
     * @brief Gets the background mean from all the points.
     * @return The reading in Greys/s.
     */
    float backgroundLevel() const;
    /*
     * @brief Sets the background mean from all the points.
     * @param The reading in Greys/s.
     */
    void backgroundLevel(const float &level);
    /*
     * @brief Gets the start time when the radioactive source is at the BLM.
     * @return Arithmetic type capable of representing times. Although not defined, this is almost
     *         always an integral value holding the number of seconds (not counting leap seconds)
     *         since 00:00, Jan 1 1970 UTC, corresponding to POSIX time.
     */
    std::time_t timeTestStart() const;
    /*
     * @brief Sets the start time when the radioactive source is at the BLM.
     * @param Arithmetic type capable of representing times. Although not defined, this is almost
     *        always an integral value holding the number of seconds (not counting leap seconds)
     *        since 00:00, Jan 1 1970 UTC, corresponding to POSIX time.
     */
    void timeTestStart(const std::time_t &timeValue);
    /*
     * @brief Gets the end time before the source is moved away from the BLM.
     * @return Arithmetic type capable of representing times. Although not defined, this is almost
     *         always an integral value holding the number of seconds (not counting leap seconds)
     *         since 00:00, Jan 1 1970 UTC, corresponding to POSIX time.
     */
    std::time_t timeTestEnd() const;
    /*
     * @brief Sets the end time before the source is moved away from the BLM.
     * @param Arithmetic type capable of representing times. Although not defined, this is almost
     *        always an integral value holding the number of seconds (not counting leap seconds)
     *        since 00:00, Jan 1 1970 UTC, corresponding to POSIX time.
     */
    void timeTestEnd(const std::time_t &timeValue);
    /*
     * @brief Gets the median of all the points with the background subtracted.
     * @return The median in Greys/s.
     */
    float signalMedian() const;
    /*
     * @brief Sets the median of all the points with the background subtracted.
     * @param The median in Greys/s.
     */
    void signalMedian(const float &mean);
    /*
     * @brief Gets the mean of all the points with the background subtracted.
     * @return The mean in Greys/s.
     */
    float signalMean() const;
    /*
     * @brief Sets the mean of all the points with the background subtracted.
     * @param The mean in Greys/s.
     */
    void signalMean(const float &mean);
    /*
     * @brief Gets the standard deviation of all the points with the background subtracted.
     * @return The standard deviation.
     */
    float signalStandardDeviation() const;
    /*
     * @brief Sets the standard deviation of all the points with the background subtracted.
     * @param The standard deviation.
     */
    void signalStandardDeviation(const float &standardDeviation);
    /*
     * @brief Gets if the signal is above the threshold, meaning the the BLM is working as expected.
     * @return True if the signal is bigger than the threshold.
     * @return False if the signal is not bigger than the threshold.
     */
    bool isSignalAboveThreshold() const;
    /*
     * @brief Sets if the signal is above the threshold, meaning the the BLM is working as expected.
     * @return True if the signal is bigger than the threshold.
     * @return False if the signal is not bigger than the threshold.
     */
    void isSignalAboveThreshold(const bool &aboveThreshold);
    /*
     * @brief Gets the names of all the BLM that are above the threshold.
     * @return The monitor names.
     */
    std::vector<std::string> blmsAboveThreshold() const;
    /*
     * @brief Sets the names of all the BLM that are above the threshold.
     * @param vector of the monitor names.
     */
    void blmsAboveThreshold(const std::vector<std::string> &blmNames);

 private:
    std::string monitorName_;
    std::string interactionPoint_;
    std::time_t timeBackgroundStart_;
    std::time_t timeBackgroundEnd_;
    float backgroundLevel_;
    std::time_t timeTestStart_;
    std::time_t timeTestEnd_;
    float signalMedian_;
    float signalMean_;
    float signalStandardDeviation_;
    bool isSignalAboveThreshold_;
    std::vector<std::string> blmsAboveThreshold_;
};

}  // namespace blmtriggeringanalysis
}  // namespace applications
}  // namespace crf
