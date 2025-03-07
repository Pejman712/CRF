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
#include <utility>

#include "Types/Types.hpp"
#include "IDataLogger.hpp"
#include "DataLogger/DataPoint.hpp"

namespace crf {
namespace utility {
namespace datalogger {

class INumericLogger : public IDataLogger {
 public:
  bool selectDB(const std::string& dbName) override = 0;
  bool dropDB(const std::string& dbName) override = 0;
  bool Export(const std::string& dbName, const std::string& measurement,
      const std::string& tag = std::string(),
      const std::pair<std::string, std::string>& timer = {std::string(), std::string()
      }) override = 0;
  /*
   * @breif writes data from a device's JointPositions into the database based on input
   * @param dbName is the user chosen name of the database
   * tag is the specific ID for the device you want to store the data froom (e.g KinovaArm2)
   * joints are the position of the joints returned in an vector of floats
   * Returns:
     *      true upon success
     *      false upon failure (e.g. device not connected, device already initialized)
   */
  virtual bool writer(const std::string& tag,
      const crf::utility::types::JointPositions& joints) = 0;
  /*
   * @breif writes data from a device's JointVelocities into the database based on input
   * @param dbName is the user chosen name of the database
   * tag is the specific ID for the device you want to store the data froom (e.g KinovaArm2)
   * joints are the velocity of the joints returned in an vector of floats
   * Returns:
     *      true upon success
     *      false upon failure (e.g. device not connected, device already initialized)
   */
  virtual bool writer(const std::string& tag,
      const crf::utility::types::JointVelocities& joints) = 0;
  /*
   * @breif writes data from a device's JointForceTorques into the database based on input
   * @param dbName is the user chosen name of the database
   * tag is the specific ID for the device you want to store the data froom (e.g KinovaArm2)
   * joints are the torque of the joints returned in an vector of floats
   * Returns:
     *      true upon success
     *      false upon failure (e.g. device not connected, device already initialized)
   */
  virtual bool writer(const std::string& tag,
      const crf::utility::types::JointForceTorques& joints) = 0;
  /*
   * @breif writes data from a device's JointAccelerations into the database based on input
   * @param dbName is the user chosen name of the database
   * tag is the specific ID for the device you want to store the data froom (e.g KinovaArm2)
   * joints are the acceleration of the joints returned in an vector of floats
   * Returns:
     *      true upon success
     *      false upon failure (e.g. device not connected, device already initialized)
   */
  virtual bool writer(const std::string& tag,
      const crf::utility::types::JointAccelerations& joints) = 0;
  /*
   * @breif writes data from a device's TaskPose into the database based on input
   * @param dbName is the user chosen name of the database
   * tag is the specific ID for the device you want to store the data froom (e.g KinovaArm2)
   * task are the position of the joints returned in an vector of floats
   * Returns:
     *      true upon success
     *      false upon failure (e.g. device not connected, device already initialized)
   */
  virtual bool writer(const std::string& tag,
      const crf::utility::types::TaskPose& task) = 0;
  /*
   * @breif writes data from a device's TaskVelocity into the database based on input
   * @param dbName is the user chosen name of the database
   * tag is the specific ID for the device you want to store the data froom (e.g KinovaArm2)
   * task are the velocity of the joints returned in an vector of floats
   * Returns:
     *      true upon success
     *      false upon failure (e.g. device not connected, device already initialized)
   */
  virtual bool writer(const std::string& tag,
      const crf::utility::types::TaskVelocity& task) = 0;
  /*
   * @breif writes data from a device's TaskForceTorque into the database based on input
   * @param dbName is the user chosen name of the database
   * tag is the specific ID for the device you want to store the data froom (e.g KinovaArm2)
   * task are the torque of the joints returned in an vector of floats
   * Returns:
     *      true upon success
     *      false upon failure (e.g. device not connected, device already initialized)
   */
  virtual bool writer(const std::string& tag,
      const crf::utility::types::TaskForceTorque& task) = 0;
  /*
   * @breif writes data from a device's TaskAcceleration into the database based on input
   * @param dbName is the user chosen name of the database
   * tag is the specific ID for the device you want to store the data froom (e.g KinovaArm2)
   * task are the acceleration of the joints returned in an vector of floats
   * Returns:
     *      true upon success
     *      false upon failure (e.g. device not connected, device already initialized)
   */
  virtual bool writer(const std::string& tag,
      const crf::utility::types::TaskAcceleration& task) = 0;
  /*
   * @breif writes data based on the chosen value to the database based on input
   * @param dbName is the user chosen name of the database
   * tag is the specific ID for the device you want to store the data froom (e.g KinovaArm2)
   * value is any input value the user wants.
   * It can be used to store all single type of information
   * Returns:
     *      true upon success
     *      false upon failure (e.g. device not connected, device already initialized)
   */
  virtual bool writer(const std::string& measurement,
      const std::string& tag, const bool& value) = 0;
  /*
   * @breif writes data based on the chosen value to the database based on input
   * @param dbName is the user chosen name of the database
   * tag is the specific ID for the device you want to store the data froom (e.g KinovaArm2)
   * value is any input value the user wants.
   * It can be used to store all single type of information
   * Returns:
     *      true upon success
     *      false upon failure (e.g. device not connected, device already initialized)
   */
  virtual bool writer(const std::string& measurement,
      const std::string& tag, const float& value) = 0;
  /*
   * @breif writes data based on the chosen value to the database based on input
   * @param dbName is the user chosen name of the database
   * tag is the specific ID for the device you want to store the data froom (e.g KinovaArm2)
   * value is any input value the user wants.
   * It can be used to store all single type of information
   * Returns:
     *      true upon success
     *      false upon failure (e.g. device not connected, device already initialized)
   */
  virtual bool writer(const std::string& measurement,
      const std::string& tag, const double& value) = 0;
  /*
   * @breif writes data based on the chosen value to the database based on input
   * @param dbName is the user chosen name of the database
   * tag is the specific ID for the device you want to store the data froom (e.g KinovaArm2)
   * value is any input value the user wants.
   * It can be used to store all single type of information
   * Returns:
     *      true upon success
     *      false upon failure (e.g. device not connected, device already initialized)
   */
  virtual bool writer(const std::string& measurement,
      const std::string& tag, const int& value) = 0;
  /*
   * @breif writes data based on the chosen value to the database based on input
   * @param dbName is the user chosen name of the database
   * tag is the specific ID for the device you want to store the data froom (e.g KinovaArm2)
   * value is any input value the user wants.
   * It can be used to store all single type of information
   * Returns:
     *      true upon success
     *      false upon failure (e.g. device not connected, device already initialized)
   */
  virtual bool writer(const std::string& measurement,
      const std::string& tag, const std::string& value) = 0;
  /*
   * @breif writes a vector of data based on the chosen value to the database based on input
   * @param dbName is the user chosen name of the database
   * tag is the specific ID for the device you want to store the data froom (e.g KinovaArm2)
   * value is any input values the user wants.
   * It can be used to store a vector of any type of information
   * Returns:
     *      true upon success
     *      false upon failure (e.g. device not connected, device already initialized)
   */
  virtual bool writer(const std::string& measurement,
      const std::string& tag, const std::vector<bool>& value) = 0;
  /*
   * @breif writes a vector of data based on the chosen value to the database based on input
   * @param dbName is the user chosen name of the database
   * tag is the specific ID for the device you want to store the data froom (e.g KinovaArm2)
   * value is any input values the user wants.
   * It can be used to store a vector of any type of information
   * Returns:
     *      true upon success
     *      false upon failure (e.g. device not connected, device already initialized)
   */
  virtual bool writer(const std::string& measurement,
      const std::string& tag, const std::vector<float>& value) = 0;
  /*
   * @breif writes a vector of data based on the chosen value to the database based on input
   * @param dbName is the user chosen name of the database
   * tag is the specific ID for the device you want to store the data froom (e.g KinovaArm2)
   * value is any input values the user wants.
   * It can be used to store a vector of any type of information
   * Returns:
     *      true upon success
     *      false upon failure (e.g. device not connected, device already initialized)
   */
  virtual bool writer(const std::string& measurement,
      const std::string& tag, const std::vector<double>& value) = 0;
  /*
   * @breif writes a vector of data based on the chosen value to the database based on input
   * @param dbName is the user chosen name of the database
   * tag is the specific ID for the device you want to store the data froom (e.g KinovaArm2)
   * value is any input values the user wants.
   * It can be used to store a vector of any type of information
   * Returns:
     *      true upon success
     *      false upon failure (e.g. device not connected, device already initialized)
   */
  virtual bool writer(const std::string& measurement,
      const std::string& tag, const std::vector<int>& value) = 0;
  /*
   * @breif writes a vector of data based on the chosen value to the database based on input
   * @param dbName is the user chosen name of the database
   * tag is the specific ID for the device you want to store the data froom (e.g KinovaArm2)
   * value is any input values the user wants.
   * It can be used to store a vector of any type of information
   * Returns:
     *      true upon success
     *      false upon failure (e.g. device not connected, device already initialized)
   */
  virtual bool writer(const std::string& measurement,
      const std::string& tag, const std::vector<std::string>& value) = 0;
 /*
  * @breif reads all the video data from the databased, based on the rules of the params
  * @param dbName is the user chosen name of the database
  * tag is the ID of the device you want to query
  * time is a pair with 2 values (start time, end time).
  * This allowes for selection of specific data at a specific time
  * Returns:
    * A vector of the struct data that contains the data the quert and also the time stamp.
   */
  virtual std::vector<dataPoint> reader(const std::string& dbName, const std::string& measurement,
      const std::string& tag = std::string(),
      const std::pair<std::string, std::string>& timer = {std::string(), std::string()}) = 0;
 /*
  * @breif sends the data string to the chosen URL.
  * @param URL is the address where the query is to be send to
  * Returns:
    *      true upon successful transmission to server
    *      false upon failure
  */
 private:
  virtual bool send() = 0;
};

}  // namespace datalogger
}  // namespace utility
}  // namespace crf
