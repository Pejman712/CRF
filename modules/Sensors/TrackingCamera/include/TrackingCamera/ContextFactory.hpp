#pragma once

/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Camarero Vera CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>

#include "librealsense2/rs.hpp"

namespace crf {
namespace sensors {
namespace realsense {

class ContextFactory {
 public:
    static rs2::context& getContext();
 private:
    static rs2::context ctx_;
};

}   // namespace realsense
}   // namespace sensors
}   // namespace cern

