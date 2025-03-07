/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <array>
#include <cmath>

#include "Laser/LaserUtils.hpp"

namespace crf {
namespace sensors {
namespace laser {

LaserUtils::LaserUtils() {
}

std::array<float, LASER_3D_DATA_SIZE> LaserUtils::CartesianToSpherical(float x, float y, float z) {
    float radius = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
    float azimuth = acos(z / radius);
    float polar = atan2(y, x);
    return {radius, azimuth, polar};
}

std::array<float, LASER_3D_DATA_SIZE> LaserUtils::CartesianToSpherical(float x, float y) {
    auto rangeThetaArray = CartesianToSpherical(x, y, .0);
    return {rangeThetaArray[0], rangeThetaArray[2]};
}

std::array<float, LASER_3D_DATA_SIZE> LaserUtils::SphericalToCartesian(float radius, float azimuth,
    float polar) {
    if (std::abs(azimuth) > M_PI || std::abs(polar) > M_PI) {
        return {};
    }
    float x = radius * sin(azimuth) * cos(polar);
    float y = radius * sin(azimuth) * sin(polar);
    float z = radius * cos(azimuth);
    return {x, y, z};
}

std::array<float, LASER_3D_DATA_SIZE> LaserUtils::SphericalToCartesian(float range, float theta) {
    return SphericalToCartesian(range, M_PI/2, theta);
}

}  // namespace laser
}  // namespace sensors
}  // namespace crf
