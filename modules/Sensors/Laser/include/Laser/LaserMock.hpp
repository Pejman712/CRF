/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>

#include "Laser/ILaser.hpp"
#include "Laser/LaserConfiguration.hpp"


namespace crf {
namespace sensors {
namespace laser {

// dont know why this is needed, GMock generator said to make a typdef
typedef pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloudType;

class LaserMock : public ILaser {
 public:
  MOCK_METHOD0(initialize,
      bool());
  MOCK_METHOD0(deinitialize,
      bool());
  MOCK_METHOD0(getPointCloud,
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr());
  MOCK_METHOD0(getConfiguration,
      std::shared_ptr<LaserConfiguration>());
};

}  // namespace laser
}  // namespace sensors
}  // namespace crf
