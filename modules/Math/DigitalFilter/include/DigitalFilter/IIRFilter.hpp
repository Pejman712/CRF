/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Jean Paul Sulca CERN BM-CEM-MRO 2023
 * 
 *  ==================================================================================================
 */

#pragma once

#include <vector>
#include "crf/expected.hpp"

namespace crf::math::digitalfilter {

struct DF2IIRFilter {
    double y = 0;
    std::vector<double> u {};
};

/**
 * @ingroup group_iir_filter
 * @brief Implementation of a IIR Filter with Direct Form II.
 * @param x Torque input
 * @param a Coefficients to calculate the new u(n) value
 * @param b Coefficients to calculate the ouput y
 * @param u Intermediate vector of the filter
 * @return struct with the ouput value and the intermediate signal u.
 * 
 * @{
 */
DF2IIRFilter direct2FormFilter(double x, std::vector<double> a,
    std::vector<double> b, std::vector<double> u = std::vector<double>());

/**@}*/

}  // namespace crf::math::digitalfilter
