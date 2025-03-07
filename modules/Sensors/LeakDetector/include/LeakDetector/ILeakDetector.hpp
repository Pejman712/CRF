#pragma once

/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Pawel Ptasznik CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

namespace crf {
namespace sensors {
namespace leakdetector {

class ILeakDetector {
 public:
    virtual ~ILeakDetector() = default;

    /*
     * Returns floating point value representing currently measured
     * leak rate in [mbar*l/s]
     * Returns 0.0 upon failure.
     */
    virtual float getLeakRate() = 0;
    /*
     * Returns floating point value representing currently measured
     * internal pressure in [mbar]
     * Returns 0.0 upon failure
     */
    virtual float getInternalPressure() = 0;
    /*
     * Sends appropriate command to the device to clear all existing errors/warnings
     * Returns TRUE upon success, and FALSE upon failure, e.g. device reply with err code,
     * or clearError command is not supported
     */
    virtual bool clearError() = 0;
};

}  // namespace leakdetector
}  // namespace sensors
}  // namespace crf
