#pragma once

/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <vector>

#include "CommonInterfaces/IInitializable.hpp"

namespace crf {
namespace sensors {
namespace xlsadapter {

class IXlsAdapter: public utility::commoninterfaces::IInitializable {
 public:
    enum Mode {
        OneWire = 1,
        TwoWires = 2
    };
    virtual ~IXlsAdapter() = default;
     /*
     * Returns floating point vector of currently measured position of the wire measured
     * if Xls Sensor in two wire mode: representing currently measured distance between both wires
     * xy positions in [mm]
     * Returns empty vector upon failure.
     */
    virtual std::vector<float> getData() = 0;
     /*
     * Sends appropriate command to the change to desired Mode
     * Returns TRUE upon success, and FALSE upon failure, e.g. device reply with err code,
     * or device not initialized
     */
    virtual bool changeMode(Mode m) = 0;
};

}  // namespace xlsadapter
}  // namespace sensors
}  // namespace crf
