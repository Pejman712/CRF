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

#include "BLMTriggeringAnalysis/BLMTriggeringResult.hpp"

namespace crf {
namespace applications {
namespace sensorvalidationresults {

class IBLMValidationResults {
 public:
    ~IBLMValidationResults() = default;

    /*
     * @brief Tells if there are results of the given BLMs.
     * @param Monitor name.
     * @return True if there is any value saved.
     * @return False otherwise.
     */
    virtual bool contains(const std::string &monitorName) = 0;
    /*
     * @brief Saves all the results of BLM in JSON format.
     * @param Monitor name.
     * @param Priority to either overwrite or ignore previous results. The lower the number the
     *        bigger the priority.
     * @param Results in JSON format.
     * @return True if it manage to save the results with the given priority.
     * @return False otherwise.
     */
    virtual bool setValues(const std::string &monitorName, uint32_t priority,
        const nlohmann::json &json) = 0;
    /*
     * @brief Retrieves all the saved values of a BLM
     * @param Monitor name.
     * @return Results in JSON format.
     */
    virtual nlohmann::json getValues(const std::string &monitorName) = 0;
    /*
     * @brief Saves the expert monitor name of a BLM.
     * @param Monitor name.
     * @param Priority to either overwrite or ignore previous results. The lower the number the
     *        bigger the priority.
     * @param Expert monitor name.
     * @return True if it manage to save the results with the given priority.
     * @return False otherwise.
     */
    virtual bool expertMonitorName(const std::string &monitorName, uint32_t priority,
        const std::string &expertMonitorName) = 0;
    /*
     * @brief Gets the expert monitor name of a BLM.
     * @param Monitor name.
     * @return A pair structure with the priority that set this value and the expert monitor name.
     */
    virtual std::pair<uint32_t, std::string> expertMonitorName(const std::string &monitorName) = 0;
    /*
     * @brief Saves the interaction point of a BLM.
     * @param Monitor name.
     * @param Priority to either overwrite or ignore previous results. The lower the number the
     *        bigger the priority.
     * @param Interaction point.
     * @return True if it manage to save the results with the given priority.
     * @return False otherwise.
     */
    virtual bool interactionPoint(const std::string &monitorName, uint32_t priority,
        const std::string &ip) = 0;
    /*
     * @brief Gets the interaction point of a BLM.
     * @param Monitor name.
     * @return A pair structure with the priority that set this value and the interaction point.
     */
    virtual std::pair<uint32_t, std::string> interactionPoint(const std::string &monitorName) = 0;
    /*
     * @brief Saves the expected DCUM of a BLM.
     * @param Monitor name.
     * @param Priority to either overwrite or ignore previous results. The lower the number the
     *        bigger the priority.
     * @param The DCUM in meters.
     * @return True if it manage to save the results with the given priority.
     * @return False otherwise.
     */
    virtual bool expectedDCUM(const std::string &monitorName, uint32_t priority,
        float expectedDCUM) = 0;
    /*
     * @brief Gets the expected DCUM of a BLM.
     * @param Monitor name.
     * @return A pair structure with the priority that set this value and the expected DCUM.
     */
    virtual std::pair<uint32_t, float> expectedDCUM(const std::string &monitorName) = 0;
    /*
     * @brief Saves the measured DCUM of a BLM.
     * @param Monitor name.
     * @param Priority to either overwrite or ignore previous results. The lower the number the
     *        bigger the priority.
     * @param The DCUM in meters.
     * @return True if it manage to save the results with the given priority.
     * @return False otherwise.
     */
    virtual bool measuredDCUM(const std::string &monitorName, uint32_t priority,
        float measuredDCUM) = 0;
    /*
     * @brief Gets the measured DCUM of a BLM.
     * @param Monitor name.
     * @return A pair structure with the priority that set this value and the measured DCUM.
     */
    virtual std::pair<uint32_t, float> measuredDCUM(const std::string &monitorName) = 0;
    /*
     * @brief Saves the reachability level of a BLM.
     * @param Monitor name.
     * @param Priority to either overwrite or ignore previous results. The lower the number the
     *        bigger the priority.
     * @param Reachability level.
     * @return True if it manage to save the results with the given priority.
     * @return False otherwise.
     */
    virtual bool reachabilityLevel(const std::string &monitorName, uint32_t priority,
        const std::string &reachibility) = 0;
    /*
     * @brief Gets the reachability level of a BLM.
     * @param Monitor name.
     * @return A pair structure with the priority that set this value and the reachability level.
     */
    virtual std::pair<uint32_t, std::string> reachabilityLevel(const std::string &monitorName) = 0;
    /*
     * @brief Saves the location of a BLM.
     * @param Monitor name.
     * @param Priority to either overwrite or ignore previous results. The lower the number the
     *        bigger the priority.
     * @param Location.
     * @return True if it manage to save the results with the given priority.
     * @return False otherwise.
     */
    virtual bool location(const std::string &monitorName, uint32_t priority,
        const std::string &location) = 0;
    /*
     * @brief Gets the location of a BLM.
     * @param Monitor name.
     * @return A pair structure with the priority that set this value and the location.
     */
    virtual std::pair<uint32_t, std::string> location(const std::string &monitorName) = 0;
    /*
     * @brief Saves the position of a BLM.
     * @param Monitor name.
     * @param Priority to either overwrite or ignore previous results. The lower the number the
     *        bigger the priority.
     * @param Position in the magnet.
     * @return True if it manage to save the results with the given priority.
     * @return False otherwise.
     */
    virtual bool position(const std::string &monitorName, uint32_t priority,
        const std::string &position) = 0;
    /*
     * @brief Gets the position of a BLM.
     * @param Monitor name.
     * @return A pair structure with the priority that set this value and the position.
     */
    virtual std::pair<uint32_t, std::string> position(const std::string &monitorName) = 0;
    /*
     * @brief Saves label ID the of a BLM.
     * @param Monitor name.
     * @param Priority to either overwrite or ignore previous results. The lower the number the
     *        bigger the priority.
     * @param Label ID.
     * @return True if it manage to save the results with the given priority.
     * @return False otherwise.
     */
    virtual bool labelID(const std::string &monitorName, uint32_t priority,
        const std::string &labelID) = 0;
    /*
     * @brief Gets the label ID of a BLM.
     * @param Monitor name.
     * @return A pair structure with the priority that set this value and the label ID.
     */
    virtual std::pair<uint32_t, std::string> labelID(const std::string &monitorName) = 0;
    /*
     * @brief Saves the QR code in the label of a BLM
     * @param Monitor name.
     * @param Priority to either overwrite or ignore previous results. The lower the number the
     *        bigger the priority.
     * @param QR code
     * @return True if it manage to save the results with the given priority.
     * @return False otherwise.
     */
    virtual bool labelQRCode(const std::string &monitorName, uint32_t priority,
        const std::string &labelQRCode) = 0;
    /*
     * @brief Gets the label QR code of a BLM.
     * @param Monitor name.
     * @return A pair structure with the priority that set this value and the label QR code.
     */
    virtual std::pair<uint32_t, std::string> labelQRCode(const std::string &monitorName) = 0;
    /*
     * @brief Tells if the label of a BLM is correct/
     * @param Monitor name.
     * @param Priority to either overwrite or ignore previous results. The lower the number the
     *        bigger the priority.
     * @param True if the label is correct, false otherwise.
     * @return True if it manage to save the results with the given priority.
     * @return False otherwise.
     */
    virtual bool isLabelCorrect(const std::string &monitorName, uint32_t priority,
        bool isLabelCorrect) = 0;
    /*
     * @brief Gets if the label of a BLM is correct.
     * @param Monitor name.
     * @return A pair structure with the priority that set this value and if the label is correct.
     */
    virtual std::pair<uint32_t, bool> isLabelCorrect(const std::string &monitorName) = 0;
    /*
     * @brief Saves the selected running sum of a BLM.
     * @param Monitor name.
     * @param Priority to either overwrite or ignore previous results. The lower the number the
     *        bigger the priority.
     * @param The running sum used for the analysis.
     * @return True if it manage to save the results with the given priority.
     * @return False otherwise.
     */
    virtual bool selectedRunningSum(const std::string &monitorName, uint32_t priority,
        const std::string &runningSum) = 0;
    /*
     * @brief Gets the selected running sum of a BLM.
     * @param Monitor name.
     * @return A pair structure with the priority that set this value and the selected running sum.
     */
    virtual std::pair<uint32_t, std::string> selectedRunningSum(const std::string &monitorName) = 0;
    /*
     * @brief Saves the irradiation time of a BLM
     * @param Monitor name.
     * @param Priority to either overwrite or ignore previous results. The lower the number the
     *        bigger the priority.
     * @param Irradiation time needed for the analysis in seconds.
     * @return True if it manage to save the results with the given priority.
     * @return False otherwise.
     */
    virtual bool irradiationTime(const std::string &monitorName, uint32_t priority,
        float time) = 0;
    /*
     * @brief Gets the irradiation time of a BLM.
     * @param Monitor name.
     * @return A pair structure with the priority that set this value and the irradiation time.
     */
    virtual std::pair<uint32_t, float> irradiationTime(const std::string &monitorName) = 0;
    /*
     * @brief Saves the threshold for the analysis of a BLM
     * @param Monitor name.
     * @param Priority to either overwrite or ignore previous results. The lower the number the
     *        bigger the priority.
     * @param Threshold in Greys/s
     * @return True if it manage to save the results with the given priority.
     * @return False otherwise.
     */
    virtual bool analysisThreshold(const std::string &monitorName, uint32_t priority,
        float threshold) = 0;
    /*
     * @brief Gets the threshold for the analysis of a BLM.
     * @param Monitor name.
     * @return A pair structure with the priority that set this value and the threshold for the
     *         analysis in Greys/s.
     */
    virtual std::pair<uint32_t, float> analysisThreshold(const std::string &monitorName) = 0;
    /*
     * @brief Saves the distance to the radioactive source of a BLM.
     * @param Monitor name.
     * @param Priority to either overwrite or ignore previous results. The lower the number the
     *        bigger the priority.
     * @param Distance to the source in meters.
     * @return True if it manage to save the results with the given priority.
     * @return False otherwise.
     */
    virtual bool distanceToSource(const std::string &monitorName, uint32_t priority,
        float distance) = 0;
    /*
     * @brief Gets the distance to the radioactive source of a BLM.
     * @param Monitor name.
     * @return A pair structure with the priority that set this value and the distance to the
     *         radioactive source.
     */
    virtual std::pair<uint32_t, float> distanceToSource(const std::string &monitorName) = 0;
    /*
     * @brief Saves triggering analysis results the of a BLM.
     * @param Monitor name.
     * @param Priority to either overwrite or ignore previous results. The lower the number the
     *        bigger the priority.
     * @param The triggering analysis results.
     * @return True if it manage to save the results with the given priority.
     * @return False otherwise.
     */
    virtual bool triggeringAnalysis(const std::string &monitorName, uint32_t priority,
        blmtriggeringanalysis::BLMTriggeringResult triggeringAnalysis) = 0;
    /*
     * @brief Gets the triggering analysis results of a BLM.
     * @param Monitor name.
     * @return A pair structure with the priority that set this value and the triggering analysis
     *         results.
     */
    virtual std::pair<uint32_t, blmtriggeringanalysis::BLMTriggeringResult> triggeringAnalysis(
        const std::string &monitorName) = 0;
};

}  // namespace sensorvalidationresults
}  // namespace applications
}  // namespace crf
