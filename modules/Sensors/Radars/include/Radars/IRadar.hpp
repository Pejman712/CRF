#pragma once

/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include "vector"
#include "complex"

#include "CommonInterfaces/IInitializable.hpp"

namespace crf {
namespace sensors {
namespace fraunhoferradar {

/**
* Interface class for radar devices.
* Known subclasses: FraunhoferRadar.
*/

class IRadar : public utility::commoninterfaces::IInitializable {
 public:
    virtual ~IRadar() = default;
    bool initialize() override = 0;
    bool deinitialize() override = 0;
    /**
    * Returns raw radar data as 2-dimensional float vector.
    * The inner vector contains fast time data, whereas the outer vector contains slow time data.
    * Length of fast time data vec is dependent on ramp length and frequency bandwidth.
    * Length of slow time data vec depends on ramp count.
    * Returns empty vector upon failure.
    *
    * Fast time - sampling a single FM signal ramp.
    * Slow time - sampling (stacking up) multiple ramps.
    */
    virtual std::vector<std::vector<float>> getFrame() = 0;
    /**
    * Returns Nyquist frequency - max observable doppler frequency across slow time samples.
    * Returns 0 upon failure
    */
    virtual float getMaxObservationFrequency() = 0;
};

}  // namespace fraunhoferradar
}  // namespace sensors
}  // namespace crf
