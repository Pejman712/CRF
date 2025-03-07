#pragma once

/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

// general
#include <string>
#include <complex>
#include <vector>
#include <iostream>
#include <cstdint>
#include <unistd.h>
#include <bitset>
// general CRF
#include "EventLogger/EventLogger.hpp"
// Xethru
#include <ModuleConnector/Data.hpp>
#include <ModuleConnector/DataRecorder.hpp>
#include <ModuleConnector/ModuleConnector.hpp>
#include <ModuleConnector/X4M200.hpp>
#include <ModuleConnector/XEP.hpp>
#include "ModuleConnector/xtid.h"
#include "Radars/XethruRadar/IXethruAdapter.hpp"

namespace crf {
namespace sensors {
namespace xethruradar {

class XethruAdapter: public IXethruAdapter {
 public:
    XethruAdapter() = delete;
    explicit XethruAdapter(const std::string& deviceFilename);
    virtual ~XethruAdapter();
    bool initialize() override;
    bool deinitialize() override;
    bool startStream(float fa1, float fa2, int sensitivity) override;
    XeThru::RespirationData getFrame() override;
    bool startXethruRecording(const std::string& DataPath) override;
    bool stopStream() override;
    bool stopXethruRecording() override;

 private:
    utility::logger::EventLogger logger_;
    std::string deviceName_;
    XeThru::ModuleConnector mc_;
    XeThru::X4M200& x4m200_;
    bool initialized_;
    bool frameReady_;
    std::string module_;
    std::string fwid_;
    XeThru::XEP& xep_;
    XeThru::DataRecorder& dataRecorder_;
    XeThru::RespirationData respirationData_;
    XeThru::DataTypes dataTypes_;    // specify to record all available data types
};

}  // namespace xethruradar
}  // namespace sensors
}  // namespace crf
