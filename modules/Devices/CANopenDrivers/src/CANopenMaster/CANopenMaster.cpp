/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Natalija Topalovic CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include <string>
#include <cstdint>

#include <lely/coapp/master.hpp>

#include "CANopenDrivers/CANopenMaster/CANopenMaster.hpp"

namespace crf::devices::canopendrivers {

void CANopenMaster::OnCommand(lely::canopen::NmtCommand cs) noexcept {
    if (cs != lely::canopen::NmtCommand::RESET_COMM) {
        lely::canopen::AsyncMaster::OnCommand(cs);
        return;
    }
    std::string path = __FILE__;
    path = path.substr(0, path.find("cpproboticframework"));
    for (uint8_t i = 0x00; i <= 0x7F; i++) {
        const char* file = lely::canopen::AsyncMaster::GetUploadFile(0x1F22, i);
        if (file == nullptr) continue;
        std::string totalPath = path + file;
        lely::canopen::AsyncMaster::SetUploadFile(0x1F22, i, totalPath.c_str());
    }
    lely::canopen::AsyncMaster::OnCommand(cs);
}

}  // namespace crf::devices::canopendrivers
