/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <array>

namespace crf {
namespace utility {
namespace visionutility {
namespace pointcloud {
namespace pointconverter {

/**
 * @brief 
 * 
 * @param x 
 * @param y 
 * @param z 
 * @return std::array<float, 3> 
 */
std::array<float, 3> cartesian2spherical(float x, float y, float z);

/**
 * @brief 
 * 
 * @param x 
 * @param y 
 * @return std::array<float, 3> 
 */
std::array<float, 3> cartesian2spherical(float x, float y);

/**
 * @brief 
 * 
 * @param radius 
 * @param azimuth 
 * @param polar 
 * @return std::array<float, 3> 
 */
std::array<float, 3> spherical2cartesian(float radius, float azimuth, float polar);

/**
 * @brief 
 * 
 * @param range 
 * @param theta 
 * @return std::array<float, 3> 
 */
std::array<float, 3> spherical2cartesian(float range, float theta);

}  // namespace pointconverter
}  // namespace pointcloud
}  // namespace visionutility
}  // namespace utility
}  // namespace crf
