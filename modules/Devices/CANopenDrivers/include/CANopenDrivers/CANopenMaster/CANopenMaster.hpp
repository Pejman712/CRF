/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Natalija Topalovic CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */
#pragma once

#include <string>
#include <cstdint>

#include <lely/coapp/master.hpp>

namespace crf::devices::canopendrivers {

/**
 * @ingroup group_canopen_master
 * @brief The CANopenMaster class inherits from the default LELY master. This
 * class only takes care of creating dynamic paths for the config files. All the
 * other functionalities are left as-is.
 *
 */
class CANopenMaster : public lely::canopen::AsyncMaster {
 public:
    using lely::canopen::AsyncMaster::AsyncMaster;

 protected:
    void OnCommand(lely::canopen::NmtCommand cs) noexcept override;
};

}  // namespace crf::devices::canopendrivers
