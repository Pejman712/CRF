/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Victor Mtsimbe Norrild CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include <chrono>
#include <string>
#include <vector>
#include <iostream>
#include <type_traits>
#include <algorithm>

#include "Types/Types.hpp"

namespace crf {
namespace utility {
namespace datalogger {

class dataPoint {
 public:
  template<class T, class = typename std::enable_if<std::is_fundamental<T>::value>::type>
  std::vector<T> cast() {
           std::vector<T> castVec(value_.size());
           std::transform(value_.begin(), value_.end(), castVec.begin(), [](const std::string& val)
           {
               return std::stod(val);
           });
       return castVec;
  }

  template<class T, class = typename std::enable_if<std::is_fundamental<T>::value>::type>
  T castFundamental() {
           std::stringstream convert(value_[0]);
           T value;
           convert >> value;
       return value;
  }

  template<class T, class = typename std::enable_if<
  std::is_same<crf::utility::types::JointPositions, T>::value
  || std::is_same<crf::utility::types::JointVelocities, T>::value
  || std::is_same<crf::utility::types::JointForceTorques, T>::value
  || std::is_same<crf::utility::types::JointAccelerations, T>::value>::type>
  T castJoints() {
           T vector(value_.size());
           for (size_t i = 0; i < value_.size(); i++) {
               vector(i) = std::stof(value_[i]);
           }

           return vector;
  }

  template<class T, class = typename std::enable_if<
  std::is_same<crf::utility::types::TaskPose, T>::value
  || std::is_same<crf::utility::types::TaskVelocity, T>::value
  || std::is_same<crf::utility::types::TaskForceTorque, T>::value
  || std::is_same<crf::utility::types::TaskAcceleration, T>::value>::type>
       T castTask() {
           T vector(6);
           for (size_t i = 0; i < 6; i++) {
               vector(i) = std::stof(value_[i]);
           }

           return vector;
  }

  /*
   * ID to differentiate between different dataPoints
   */
  std::string id_;
  /*
   * Timestamp for the  datapoint stored in type string
   */
  std::string time_;
  /*
   * The value read from the query to the DB stored in a vector of type template
   */
  std::vector<std::string> value_;
};

}  // namespace datalogger
}  // namespace utility
}  // namespace crf
