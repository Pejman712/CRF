#pragma once

/* Ã‚Â© Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */
#include <string>
#include "CommonInterfaces/IInitializable.hpp"

namespace XeThru {
class RespirationData;
}

namespace crf {
namespace sensors {
namespace xethruradar {

class IXethruAdapter: public utility::commoninterfaces::IInitializable {
 public:
    virtual bool initialize() = 0;
    virtual bool deinitialize() = 0;
    virtual bool startStream(float fa1, float fa2, int sensitivity) = 0;
    virtual XeThru::RespirationData getFrame() = 0;
    virtual bool startXethruRecording(const std::string& DataPath) = 0;
    virtual bool stopStream() = 0;
    virtual bool stopXethruRecording() = 0;
};

}  // namespace xethruradar
}  // namespace sensors
}  // namespace crf
