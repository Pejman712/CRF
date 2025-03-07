#pragma once
/* Â© Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <thread>
#include <iostream>
#include <cstring>
#include <vector>
#include <opencv2/core.hpp>

#include <pcl/filters/extract_indices.h>
#include <pcl/common/geometry.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include <nlohmann/json.hpp>

#include "IPC/IPC.hpp"
#include "EventLogger/EventLogger.hpp"

#include "LaserCommunicationPoint/LaserCommunicationPoint.hpp"

#include "Types/TaskTypes/TaskPose.hpp"

#include "RobotBase/RobotBaseConfiguration.hpp"

#include "PersonFollower/IPersonDetector.hpp"
#include "PersonFollower/IPersonTracker.hpp"

namespace crf {
namespace applications {
namespace personfollower {

class PersonTracker : public IPersonTracker {
 public:
    PersonTracker(std::shared_ptr<IPC> personTrackerOutputIpc,
      std::shared_ptr<IPersonDetector> detector,
      const nlohmann::json &LaserParams,
      std::shared_ptr<robots::robotbase::RobotBaseConfiguration> robotBaseConfig);
    PersonTracker(const PersonTracker& other) = delete;
    PersonTracker(PersonTracker&& other) = delete;
    PersonTracker() = delete;
    ~PersonTracker() override;
    bool initialize() override;
    bool deinitialize() override;
    PersonCentroid trackPerson(const utility::types::TaskPose& cameraPose) override;
    int getCameraDisplacement() override;

 private:
    void getFrontLaserScan(const utility::types::TaskPose& cameraPose);
    void filterRobotSelfReflections(const utility::types::TaskPose& cameraPose);
    void addEdgeInformation();
    std::vector<UnassignedCentroid> legPatternRecognition();
    bool inspectElement(const pcl::PointXYZRGBA inspectedPoint, const int expectedValue);
    int inspectRegion(const int startPoint, const int expectedValue);
    void removeDetectedPatternsFromPointCloud(const int startPoint, const int endPoint);
    void createInputCentroid(int startPoint, int secondPoint, int thirdPoint, int endPoint,
      std::vector<UnassignedCentroid>* unassignedCentroidVector);
    void createInputCentroid(int startPoint, int endPoint,
      std::vector<UnassignedCentroid>* unassignedCentroidVector);
    void updatePersonLocation(std::vector<UnassignedCentroid>* unassignedCentroidVector);
    bool readConfig();

    float laserDataPointAmplitudeDifference;
    uint8_t maxDistanceBetweenPts;
    uint8_t laserFrontRangePts;
    std::shared_ptr<IPC> personTrackerOutputIpc_;
    std::shared_ptr<IPersonDetector> detector_;
    int imageWidth_;
    nlohmann::json jConfig_;
    PersonCentroid personPoint_;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloud_;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr remOutPointCloud_;
    bool initialized_;
    robots::robotbase::RobotParameters robotParameters_;
    utility::logger::EventLogger logger_;
};

}  // namespace personfollower
}  // namespace applications
}  // namespace crf
