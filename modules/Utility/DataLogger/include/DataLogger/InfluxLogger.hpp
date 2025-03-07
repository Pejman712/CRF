/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Victor Mtsimbe Norrild CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <type_traits>
#include <typeinfo>
#include <utility>

#include "Types/Types.hpp"
#include "DataLogger/INumericLogger.hpp"
#include "IDataLogger.hpp"

namespace crf {
namespace utility {
namespace datalogger {

class InfluxLogger : public INumericLogger {
 public:
  explicit InfluxLogger(const int& lineCounterMax = 100000, const float& timerMax = 60.0) :
     t1_(&InfluxLogger::senderThread, this),
     lineCounterMax_(lineCounterMax),
     timerMax_(timerMax),
     stopThread_(true) {
         std::cout << "DataLogger is running" << std::endl;
      }
  bool selectDB(const std::string& dbName) override;
  bool dropDB(const std::string& dbName) override;
  bool Export(const std::string& dbName, const std::string& measurement,
      const std::string& tag = std::string(),
      const std::pair<std::string, std::string>& timer = {std::string(), std::string()}) override;

  bool writer(const std::string& tag,
      const crf::utility::types::JointPositions& joints) override;
  bool writer(const std::string& tag,
      const crf::utility::types::JointVelocities& joints) override;
  bool writer(const std::string& tag,
      const crf::utility::types::JointForceTorques& joints) override;
  bool writer(const std::string& tag,
      const crf::utility::types::JointAccelerations& joints) override;
  bool writer(const std::string& tag,
     const crf::utility::types::TaskPose& task) override;
  bool writer(const std::string& tag,
      const crf::utility::types::TaskVelocity& task) override;
  bool writer(const std::string& tag,
      const crf::utility::types::TaskForceTorque& task) override;
  bool writer(const std::string& tag,
      const crf::utility::types::TaskAcceleration& task) override;
  bool writer(const std::string& measurement, const std::string& tag,
      const bool& value) override;
  bool writer(const std::string& measurement, const std::string& tag,
      const float& value) override;
  bool writer(const std::string& measurement, const std::string& tag,
    const double& value) override;
  bool writer(const std::string& measurement, const std::string& tag,
      const int& value) override;
  bool writer(const std::string& measurement, const std::string& tag,
      const std::string& value) override;
  bool writer(const std::string& measurement, const std::string& tag,
      const std::vector<bool>& value) override;
  bool writer(const std::string& measurement, const std::string& tag,
      const std::vector<float>& value) override;
  bool writer(const std::string& measurement, const std::string& tag,
      const std::vector<double>& value) override;
  bool writer(const std::string& measurement, const std::string& tag,
      const std::vector<int>& value) override;
  bool writer(const std::string& measurement, const std::string& tag,
      const std::vector<std::string>& value) override;

  std::vector<dataPoint> reader(const std::string& dbName, const std::string& measurement,
      const std::string& tag = std::string(),
      const std::pair<std::string, std::string>& timer = {std::string(), std::string()}) override;

  virtual ~InfluxLogger();


 private:
  bool send();
  void senderThread();
  std::mutex mtx_;
  std::thread t1_;
  std::string currentdb_  = "";
  std::string query_ = "";
  int lineCounter_   = 0;
  int lineCounterMax_;
  float timerMax_;
  bool stopThread_;
  bool newDB_;
};

}  // namespace datalogger
}  // namespace utility
}  // namespace crf
