/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Author: Jean Paul Sulca CERN BM-CEM-MRO 2023
 * 
 *  ==================================================================================================
 */

#include <numeric>

#include "DigitalFilter/IIRFilter.hpp"

namespace crf::math::digitalfilter {

DF2IIRFilter direct2FormFilter(double x, std::vector<double> a,
    std::vector<double> b, std::vector<double> uPrev) {
    std::size_t n = b.size();
    if (a.empty() || b.empty() || (a.size() != n) || (uPrev.size() > 0 && (uPrev.size() != n-1)))
        throw std::invalid_argument("One of the inputs has a wrong size");
    if (a[0] != 1)
        throw std::invalid_argument("Coefficient a0 must be equal to 1");

    DF2IIRFilter result;
    // If u is empty -> Obtain u initial from condition uNew = u[0] = u[1] = ...
    if (uPrev.size() == 0) {
        double uNew = x / std::reduce(a.begin(), a.end());
        result.u = std::vector<double>(n-1, uNew);
        for (std::size_t i = 0; i < n; i++) {
            result.y += b[i] * uNew;
        }
    } else {
        double uNew = x;
        for (std::size_t i = 0; i < n - 1; i++) {
            uNew -= a[i+1]*uPrev[i];
            result.y += b[i+1]*uPrev[i];
        }
        result.y += b[0]*uNew;
        result.u = {uNew};
        result.u.insert(result.u.end(), uPrev.begin(), uPrev.end()-1);
    }
    return result;
}

}  // namespace crf::math::digitalfilter
