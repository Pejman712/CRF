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

namespace crf {
namespace applications {
namespace sensorvalidationresults {

class IPMIValidationResults {
 public:
    ~IPMIValidationResults() = default;

    /*
     * @brief Tells if there are results of the given PMIs.
     * @param Name.
     * @return True if there is any value saved.
     * @return False otherwise.
     */
    virtual bool contains(const std::string &name) = 0;
    /*
     * @brief Saves all the results of PMI in JSON format.
     * @param Name.
     * @param Priority to either overwrite or ignore previous results. The lower the number the
     *        bigger the priority.
     * @param Results in JSON format.
     * @return True if it manage to save the results with the given priority.
     * @return False otherwise.
     */
    virtual bool setValues(const std::string &name, uint32_t priority,
        const nlohmann::json &json) = 0;
    /*
     * @brief Retrieves all the saved values of a PMI
     * @param Name.
     * @return Results in JSON format.
     */
    virtual nlohmann::json getValues(const std::string &name) = 0;
    /*
     * @brief Saves the expert name of a PMI.
     * @param name.
     * @param Priority to either overwrite or ignore previous results. The lower the number the
     *        bigger the priority.
     * @param Expert name.
     * @return True if it manage to save the results with the given priority.
     * @return False otherwise.
     */
    virtual bool expertName(const std::string &name, uint32_t priority,
        const std::string &expertName) = 0;
    /*
     * @brief Gets the expert name of a PMI.
     * @param name.
     * @return A pair structure with the priority that set this value and the expert name.
     */
    virtual std::pair<uint32_t, std::string> expertName(const std::string &name) = 0;
    /*
     * @brief Saves the sector of a PMI.
     * @param name.
     * @param Priority to either overwrite or ignore previous results. The lower the number the
     *        bigger the priority.
     * @param Sector.
     * @return True if it manage to save the results with the given priority.
     * @return False otherwise.
     */
    virtual bool sector(const std::string &name, uint32_t priority,
        const std::string &sector) = 0;
    /*
     * @brief Gets the sector of a PMI.
     * @param name.
     * @return A pair structure with the priority that set this value and the sector.
     */
    virtual std::pair<uint32_t, std::string> sector(const std::string &name) = 0;
    /*
     * @brief Saves the expected DCUM of a PMI.
     * @param name.
     * @param Priority to either overwrite or ignore previous results. The lower the number the
     *        bigger the priority.
     * @param The DCUM in meters.
     * @return True if it manage to save the results with the given priority.
     * @return False otherwise.
     */
    virtual bool expectedDCUM(const std::string &name, uint32_t priority,
        float expectedDCUM) = 0;
    /*
     * @brief Gets the expected DCUM of a PMI.
     * @param name.
     * @return A pair structure with the priority that set this value and the expected DCUM.
     */
    virtual std::pair<uint32_t, float> expectedDCUM(const std::string &name) = 0;
    /*
     * @brief Saves the measured DCUM of a PMI.
     * @param name.
     * @param Priority to either overwrite or ignore previous results. The lower the number the
     *        bigger the priority.
     * @param The DCUM in meters.
     * @return True if it manage to save the results with the given priority.
     * @return False otherwise.
     */
    virtual bool measuredDCUM(const std::string &name, uint32_t priority,
        float measuredDCUM) = 0;
    /*
     * @brief Gets the measured DCUM of a PMI.
     * @param name.
     * @return A pair structure with the priority that set this value and the measured DCUM.
     */
    virtual std::pair<uint32_t, float> measuredDCUM(const std::string &name) = 0;
    /*
     * @brief Saves label ID the of a PMI.
     * @param name.
     * @param Priority to either overwrite or ignore previous results. The lower the number the
     *        bigger the priority.
     * @param Label ID.
     * @return True if it manage to save the results with the given priority.
     * @return False otherwise.
     */
    virtual bool labelID(const std::string &name, uint32_t priority,
        const std::string &labelID) = 0;
    /*
     * @brief Gets the label ID of a PMI.
     * @param name.
     * @return A pair structure with the priority that set this value and the label ID.
     */
    virtual std::pair<uint32_t, std::string> labelID(const std::string &name) = 0;
    /*
     * @brief Saves the QR code in the label of a PMI
     * @param name.
     * @param Priority to either overwrite or ignore previous results. The lower the number the
     *        bigger the priority.
     * @param QR code
     * @return True if it manage to save the results with the given priority.
     * @return False otherwise.
     */
    virtual bool labelQRCode(const std::string &name, uint32_t priority,
        const std::string &labelQRCode) = 0;
    /*
     * @brief Gets the label QR code of a PMI.
     * @param name.
     * @return A pair structure with the priority that set this value and the label QR code.
     */
    virtual std::pair<uint32_t, std::string> labelQRCode(const std::string &name) = 0;
    /*
     * @brief Tells if the label of a PMI is correct/
     * @param name.
     * @param Priority to either overwrite or ignore previous results. The lower the number the
     *        bigger the priority.
     * @param True if the label is correct, false otherwise.
     * @return True if it manage to save the results with the given priority.
     * @return False otherwise.
     */
    virtual bool isLabelCorrect(const std::string &name, uint32_t priority,
        bool isLabelCorrect) = 0;
    /*
     * @brief Gets if the label of a PMI is correct.
     * @param name.
     * @return A pair structure with the priority that set this value and if the label is correct.
     */
    virtual std::pair<uint32_t, bool> isLabelCorrect(const std::string &name) = 0;
    /*
     * @brief Saves the irradiation time of a PMI
     * @param name.
     * @param Priority to either overwrite or ignore previous results. The lower the number the
     *        bigger the priority.
     * @param Irradiation time needed for the analysis in seconds.
     * @return True if it manage to save the results with the given priority.
     * @return False otherwise.
     */
    virtual bool irradiationTime(const std::string &name, uint32_t priority,
        float time) = 0;
    /*
     * @brief Gets the irradiation time of a PMI.
     * @param name.
     * @return A pair structure with the priority that set this value and the irradiation time.
     */
    virtual std::pair<uint32_t, float> irradiationTime(const std::string &name) = 0;
    /*
     * @brief Saves the start time of the test of a PMI
     * @param name.
     * @param Priority to either overwrite or ignore previous results. The lower the number the
     *        bigger the priority.
     * @param start time of the test in POSIX format
     * @return True if it manage to save the results with the given priority.
     * @return False otherwise.
     */
    virtual bool timeTestStart(const std::string &name, uint32_t priority,
        std::time_t time) = 0;
    /*
     * @brief Gets the start time of the test of a PMI.
     * @param name.
     * @return A pair structure with the priority that set this value and the start time of the
     *         test.
     */
    virtual std::pair<uint32_t, std::time_t> timeTestStart(const std::string &name) = 0;
    /*
     * @brief Saves the end time of the test of a PMI
     * @param name.
     * @param Priority to either overwrite or ignore previous results. The lower the number the
     *        bigger the priority.
     * @param end time of the test in POSIX format
     * @return True if it manage to save the results with the given priority.
     * @return False otherwise.
     */
    virtual bool timeTestEnd(const std::string &name, uint32_t priority,
        std::time_t time) = 0;
    /*
     * @brief Gets the end time of the test of a PMI.
     * @param name.
     * @return A pair structure with the priority that set this value and the end time of the
     *         test.
     */
    virtual std::pair<uint32_t, std::time_t> timeTestEnd(const std::string &name) = 0;
};

}  // namespace sensorvalidationresults
}  // namespace applications
}  // namespace crf
