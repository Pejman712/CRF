/* © Copyright CERN 2021.  All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>
#include <vector>

#include "BLMTriggeringAnalysis/BLMTriggeringResult.hpp"

namespace crf {
namespace applications {
namespace blmtriggeringanalysis {

class IBLMTriggeringAnalysis{
 public:
    ~IBLMTriggeringAnalysis() = default;

    /*
     * @brief Extracts the readings of every BLM from an interaction point, and saves them as
     *        background levels for the triggering analysis. These values are suppose to be the
     *        radiation that the sensors get from its surroundings when there is no radioactive
     *        source nearby.
     * @param The interaction point as a string.
     * @return True if it manage to get them all.
     * @return False if it failed.
     */
    virtual bool calculateBackgroundLevels(const std::string &ip) = 0;
    /*
     * @brief Executes the triggering analysis of all the BLM of an interaction point. It has into
     *        account the background levels. 
     * @param The interaction point that will be checked.
     * @return The results of each BLM with the BLMTriggeringResult structure.
     */
    virtual std::vector<BLMTriggeringResult> executeInteractionPointTriggeringAnalysis(
        const std::string &ip) = 0;
    /*
     * @brief Executes the triggering analysis of one BLM according to its name. It has into
     *        account the background levels. 
     * @param The name of the BLM.
     * @return The result with the BLMTriggeringResult structure.
     */
    virtual BLMTriggeringResult executeSpecificTriggeringAnalysis(const std::string &blmName) = 0;
    /*
     * @brief Gets the current raw value given by a specific BLM.
     * @param The name of the BLM.
     * @return radiation level in Greys/s.
     */
    virtual float getBLMValueWithoutBackground(const std::string &blmName) = 0;
};

}  // namespace blmtriggeringanalysis
}  // namespace applications
}  // namespace crf
