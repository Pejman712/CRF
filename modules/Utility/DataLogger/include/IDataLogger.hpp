/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Victor Mtsimbe Norrild CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>
#include <utility>

namespace crf {
namespace utility {
namespace datalogger {

class IDataLogger {
 public:
   /*
    * @breif Construct a new Database
    * @param dbName is the user chosen name of the database
    * Returns:
     *      true upon success
     *      false upon failure (e.g. device not connected, device already initialized)
    */
  virtual bool selectDB(const std::string& dbName) = 0;
   /*
    * @breif Drop/destroy an entire Database (will also delete all series)
    * @param dbName is the user chosen name of the database
    * Returns:
     *      true upon success
     *      false upon failure (e.g. device not connected, device already initialized)
    */
  virtual bool dropDB(const std::string& dbName) = 0;
   /*
    * @breif exports all data from a chosen series that followes the rules of the tag
    * @param dbName is the user chosen name of the database
    * measurement is the name of data stored (also called a series)
    * tag is the way to select which specific data you want (se manual at influx webside)
    * Returns:
     *      true upon success
     *      false upon failure (e.g. device not connected, device already initialized)
    */
  virtual bool Export(const std::string& dbName, const std::string& measurement,
      const std::string& tag,
      const std::pair<std::string, std::string>& timer) = 0;
};

}  // namespace datalogger
}  // namespace utility
}  // namespace crf
