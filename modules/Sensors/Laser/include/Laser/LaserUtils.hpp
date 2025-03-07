/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include <array>

#define LASER_3D_DATA_SIZE 3

namespace crf {
namespace sensors {
namespace laser {

class LaserUtils {
 public:
    static std::array<float, LASER_3D_DATA_SIZE> CartesianToSpherical(float x, float y, float z);
    static std::array<float, LASER_3D_DATA_SIZE> CartesianToSpherical(float x, float y);
    static std::array<float, LASER_3D_DATA_SIZE> SphericalToCartesian(float radius, float azimuth,
        float polar);
    static std::array<float, LASER_3D_DATA_SIZE> SphericalToCartesian(float range, float theta);
 private:
    LaserUtils();
};

}  // namespace laser
}  // namespace sensors
}  // namespace crf
