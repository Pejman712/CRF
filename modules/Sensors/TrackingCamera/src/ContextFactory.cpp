/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Camarero Vera CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>

#include "TrackingCamera/ContextFactory.hpp"

namespace crf {
namespace sensors {
namespace realsense {

rs2::context& ContextFactory::getContext() {return ctx_;}

rs2::context ContextFactory::ctx_ = rs2::context();

}   // namespace realsense
}   // namespace sensors
}   // namespace cern

