/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <algorithm>
#include <iostream>
#include <cstdlib>
#include <vector>
#include <numeric>
#include <cmath>
#include <fstream>

#include "LaserCommunicationPoint/LaserPacket.hpp"
#include "PersonFollower/PersonTracker.hpp"

#define MAX_DISAPPEARED_FRAMES 20
#define MAX_DISTANCE_TO_NEXT_CENTROID 0.3
#define START_OF_OBJECT 16711680
#define END_OF_OBJECT 65280

#define DEFAULT_ROBOT_HEIGHT 0.25  // this is temp solution, robot config don't specify robot height

namespace crf {
namespace applications {
namespace personfollower {

PersonTracker::PersonTracker(std::shared_ptr<IPC> personTrackerOutputIpc,
    std::shared_ptr<IPersonDetector> detector,
    const nlohmann::json &LaserParams,
    std::shared_ptr<robots::robotbase::RobotBaseConfiguration> robotBaseConfig):
    personTrackerOutputIpc_(personTrackerOutputIpc),
    detector_(detector),
    imageWidth_(0),
    jConfig_(LaserParams),
    personPoint_(),
    pointCloud_(new pcl::PointCloud<pcl::PointXYZRGBA>),
    remOutPointCloud_(new pcl::PointCloud<pcl::PointXYZRGBA>),
    initialized_(false),
    robotParameters_(robotBaseConfig->getRobotParameters()),
    logger_("PersonTracker") {
      logger_->debug("CTor");
      bool isConfigRead = readConfig();
      if (!isConfigRead) {
          throw std::invalid_argument("Could not read config");
      }
}

PersonTracker::~PersonTracker() {
    logger_->debug("DTor");
    deinitialize();
}

bool PersonTracker::initialize() {
    logger_->debug("initialize");
    if (initialized_) {
        logger_->debug("PersonDetector already initialized");
        return false;
    }
    if (!detector_->initialize()) {
        logger_->debug("PersonDetector could not be initialized");
        return false;
    } else if (!personTrackerOutputIpc_->open()) {
        logger_->debug("IPC reader could not be opened");
        return false;
    }
    imageWidth_ = detector_->getImageCenterWidth();
    initialized_ = true;
    return true;
}

bool PersonTracker::deinitialize() {
    logger_->debug("deinitialize");
    if (!initialized_) {
        logger_->debug("PersonDetector already deinitialized");
        return false;
    }
    personTrackerOutputIpc_->close();
    detector_->deinitialize();
    initialized_ = false;
    return true;
}

PersonCentroid PersonTracker::trackPerson(const utility::types::TaskPose& cameraPose) {
    logger_->debug("trackPerson");
    pointCloud_->clear();
    remOutPointCloud_->clear();
    getFrontLaserScan(cameraPose);
    if (!pointCloud_) {
        return personPoint_;
    }
    addEdgeInformation();
    std::vector<UnassignedCentroid> unassignedCentroidVector = legPatternRecognition();
    updatePersonLocation(&unassignedCentroidVector);
    return personPoint_;
}

int PersonTracker::getCameraDisplacement() {
    logger_->debug("getCameraDisplacement");
    cv::Rect2d detectorData = detector_->getPersonBoundingBox();
    logger_->debug("x: {}, y: {}, width: {}, height: {}, displacement: {}, imageCenterWidth: {}",
      detectorData.x,
      detectorData.y,
      detectorData.width,
      detectorData.height,
      detectorData.x + detectorData.width/2 - imageWidth_,
      imageWidth_);
    if (detectorData.empty()) {
        return 666;
    }
    return detectorData.x + detectorData.width/2 - imageWidth_;
}

void PersonTracker::getFrontLaserScan(const utility::types::TaskPose& cameraPose) {
    logger_->debug("getFrontLaserScan");
    Packets::PacketHeader header;
    Packets::LaserPacket laserPacket;
    std::string buf;
    personTrackerOutputIpc_->read(buf, header);
    if (header.type() != Packets::LASER_PACKET_TYPE) {
        logger_->warn("Wrong packet type!");
        pointCloud_ = nullptr;
        return;
    }
    laserPacket.deserialize(buf);
    if (laserPacket.pointCloud.empty()) {
        logger_->warn("Got empty data");
        pointCloud_ = nullptr;
        return;
    }
    for (size_t i = laserFrontRangePts;
      i < laserPacket.pointCloud.points.size() - laserFrontRangePts; i++) {
        pointCloud_->points.push_back(laserPacket.pointCloud.points[i]);
    }
    return;
}

void PersonTracker::addEdgeInformation() {
    logger_->debug("generateEdgeMatrix");
    pcl::PointXYZRGBA referencePoint;
    referencePoint.x = 0;
    referencePoint.y = 0;
    referencePoint.z = 0;
    float euclideanDistancePoint1, euclideanDistancePoint2;
    for (size_t i = 0; i < pointCloud_->points.size()-1; i++) {
        euclideanDistancePoint1 = pcl::geometry::distance(pointCloud_->points[i], referencePoint);
        euclideanDistancePoint2 = pcl::geometry::distance(pointCloud_->points[i+1], referencePoint);
        /* [i+1] point is marked red if [i] (previous) point is further away. This is usually
        "start" of person leg cluster */
        if (euclideanDistancePoint1 - euclideanDistancePoint2 >
          laserDataPointAmplitudeDifference) {
            uint8_t r = 255;
            uint8_t g = 0;
            uint8_t b = 0;
            uint32_t rgb = ((int)r) << 16 | ((int)g) << 8 | ((int)b); // NOLINT
            pointCloud_->points[i+1].rgba = rgb;
        /* [i] point is marked green if it is closer than [i+1] (next) point. This is usually
        "end" of person leg cluster */
        } else if (euclideanDistancePoint1 - euclideanDistancePoint2 <
          - laserDataPointAmplitudeDifference) {
            uint8_t r = 0;
            uint8_t g = 255;
            uint8_t b = 0;
            uint32_t rgb = ((int)r) << 16 | ((int)g) << 8 | ((int)b); // NOLINT
            pointCloud_->points[i].rgba = rgb;
        }
    }
    return;
}

std::vector<UnassignedCentroid> PersonTracker::legPatternRecognition() {
    logger_->debug("legPatternRecognition");
    std::vector<UnassignedCentroid> unassignedCentroidVector;
    // 1. find LA patterns
    for (size_t x = 0; x < pointCloud_->points.size(); x++) {
        if (!inspectElement(pointCloud_->points[x], START_OF_OBJECT)) {
            continue;
        }
        int secondPoint = inspectRegion(x, END_OF_OBJECT);
        if (secondPoint == 0) {
            continue;
        }
        int thirdPoint = inspectRegion(secondPoint, START_OF_OBJECT);
        if (thirdPoint == 0) {
            continue;
        }
        int endPoint = inspectRegion(thirdPoint, END_OF_OBJECT);
        if (endPoint == 0) {
            continue;
        }
        createInputCentroid(x, secondPoint, thirdPoint, endPoint, &unassignedCentroidVector);
        removeDetectedPatternsFromPointCloud(x, endPoint);
    }
    // 2. find FS patterns
    for (size_t x = 0; x < pointCloud_->points.size(); x++) {
        if (!inspectElement(pointCloud_->points[x], START_OF_OBJECT)) {
            continue;
        }
        int secondPoint = inspectRegion(x, END_OF_OBJECT);
        if (secondPoint == 0) {
            secondPoint = inspectRegion(x, START_OF_OBJECT);
            if (secondPoint == 0) {
                continue;
            }
        }
        int endPoint = inspectRegion(secondPoint, END_OF_OBJECT);
        if (endPoint == 0) {
            continue;
        }
        logger_->debug("FS detected");
        createInputCentroid(x, endPoint, &unassignedCentroidVector);
        removeDetectedPatternsFromPointCloud(x, endPoint);
    }
    // 3. find  LS patterns (the least robust pattern)
    for (size_t x = 0; x < pointCloud_->points.size(); x++) {
        if (!inspectElement(pointCloud_->points[x], START_OF_OBJECT)) {
            continue;
        }
        int endPoint = inspectRegion(x, END_OF_OBJECT);
        if (endPoint == 0) {
            continue;
        }
        logger_->debug("LS detected");
        createInputCentroid(x, endPoint, &unassignedCentroidVector);
        removeDetectedPatternsFromPointCloud(x, endPoint);
    }
    return unassignedCentroidVector;
}

bool PersonTracker::inspectElement(const pcl::PointXYZRGBA inspectedPoint, const int expectedVal) {
    // logger_->debug("inspectElement");
    if (inspectedPoint.rgba == static_cast<uint32_t>(expectedVal)) {
        return true;
    }
    return false;
}

int PersonTracker::inspectRegion(const int startPoint, const int expectedValue) {
    // logger_->debug("inspectRegion");
    for (int i = startPoint+1; i < startPoint + maxDistanceBetweenPts; i++) {
        if (pointCloud_->points[i].rgba == static_cast<uint32_t>(expectedValue)) {
            return i;
        }
    }
    return 0;
}

void PersonTracker::removeDetectedPatternsFromPointCloud(
  const int startPoint, const int endPoint) {
    logger_->debug("removeDetectedPatternsFromPointCloud");
    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;
    uint32_t rgb = ((int)r) << 16 | ((int)g) << 8 | ((int)b); // NOLINT
    for (int i = startPoint; i < endPoint; i++) {
        pointCloud_->points[i].rgba = rgb;
    }
    return;
}

void PersonTracker::createInputCentroid(int startPoint, int secondPoint, int thirdPoint,
  int endPoint, std::vector<UnassignedCentroid>* unassignedCentroidVector) {
    logger_->debug("createInputCentroid");
    pcl::CentroidPoint<pcl::PointXYZRGBA> newCentroid;
    pcl::PointXYZRGBA centroidPoint;
    UnassignedCentroid unassignedNewCentroid;
    for (int i = startPoint; i < secondPoint; i++) {
      remOutPointCloud_->points.push_back(pointCloud_->points[i]);
      logger_->debug("point: {} x, {} y, z {} ",
      pointCloud_->points[i].x, pointCloud_->points[i].y, pointCloud_->points[i].z);
    }
    for (int i = thirdPoint; i < endPoint; i++) {
      remOutPointCloud_->points.push_back(pointCloud_->points[i]);
      logger_->debug("point: {} x, {} y, z {} ",
      pointCloud_->points[i].x, pointCloud_->points[i].y, pointCloud_->points[i].z);
    }
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGBA> outlierRemoval;
    outlierRemoval.setInputCloud(remOutPointCloud_);
    outlierRemoval.setRadiusSearch(0.5);
    outlierRemoval.setMinNeighborsInRadius(4);
    outlierRemoval.filter(*remOutPointCloud_);
    for (size_t i = 0; i < remOutPointCloud_->points.size(); i++) {
      newCentroid.add(remOutPointCloud_->points[i]);
    }
    newCentroid.get(centroidPoint);
    logger_->debug("centroidPoint: {} x, {} y, z {} ",
    centroidPoint.x, centroidPoint.y, centroidPoint.z);
    unassignedNewCentroid.centroidPoint = centroidPoint;
    unassignedNewCentroid.assigned = false;
    unassignedCentroidVector->push_back(unassignedNewCentroid);
    return;
}

void PersonTracker::createInputCentroid(int startPoint, int endPoint,
  std::vector<UnassignedCentroid>* unassignedCentroidVector) {
    logger_->debug("createInputCentroid");
    pcl::CentroidPoint<pcl::PointXYZRGBA> newCentroid;
    pcl::PointXYZRGBA centroidPoint;
    UnassignedCentroid unassignedNewCentroid;
    for (int i = startPoint; i < endPoint; i++) {
        newCentroid.add(pointCloud_->points[i]);
    }
    newCentroid.get(centroidPoint);
    unassignedNewCentroid.centroidPoint = centroidPoint;
    unassignedNewCentroid.assigned = false;
    unassignedCentroidVector->push_back(unassignedNewCentroid);
    return;
}

void PersonTracker::updatePersonLocation(
  std::vector<UnassignedCentroid>* unassignedCentroidVector) {
    logger_->debug("updatePersonLocation");
    pcl::PointXYZRGBA referencePoint;
    referencePoint.x = 0;
    referencePoint.y = 0;
    referencePoint.z = 0;
    if (personPoint_.dissapeared >= MAX_DISAPPEARED_FRAMES) {
        personPoint_.objectID = -1;
        personPoint_.dissapeared = 0;
        personPoint_.centroidPoint = referencePoint;
        personPoint_.euclideanDistance = 0;
        logger_->debug("Destroying the personPoint.");
        return;
    }
    if (unassignedCentroidVector->empty()) {
        personPoint_.dissapeared += 1;
        logger_->debug("No patterns in FOV found");
        return;
    }
    if (personPoint_.objectID == -1) {
        std::vector<float> yAxisVector;
        for (size_t i = 0; i < unassignedCentroidVector->size(); i++) {
            yAxisVector.push_back(std::abs(unassignedCentroidVector->at(i).centroidPoint.y));
        }
        int minValue = std::min_element(
          yAxisVector.begin(), yAxisVector.end()) - yAxisVector.begin();
        logger_->debug("PersonPoint has been initialized");
        personPoint_.objectID = 1;
        personPoint_.dissapeared = 0;
        personPoint_.centroidPoint = unassignedCentroidVector->at(minValue).centroidPoint;
        personPoint_.euclideanDistance = pcl::geometry::distance(
          personPoint_.centroidPoint, referencePoint);
        return;
    }
    std::vector<float> distanceDifferenceVector;
    for (size_t i = 0; i < unassignedCentroidVector->size(); i++) {
        distanceDifferenceVector.push_back(
          pcl::geometry::distance(
            personPoint_.centroidPoint, unassignedCentroidVector->at(i).centroidPoint));
    }
    int minValue = std::min_element(
      distanceDifferenceVector.begin(),
      distanceDifferenceVector.end()) - distanceDifferenceVector.begin();
    logger_->debug("The closest point is {} m away", distanceDifferenceVector[minValue]);
    if (distanceDifferenceVector[minValue] < MAX_DISTANCE_TO_NEXT_CENTROID) {
        personPoint_.dissapeared = 0;
        personPoint_.centroidPoint = unassignedCentroidVector->at(minValue).centroidPoint;
        personPoint_.euclideanDistance = pcl::geometry::distance(
          personPoint_.centroidPoint, referencePoint);
        return;
    }
    logger_->debug("all identified centroids are too far.");
    personPoint_.dissapeared += 1;
    return;
}

bool PersonTracker::readConfig() {
    try {
        laserFrontRangePts = jConfig_.at("LASER_FRONT_RANGE_BINS").get<uint8_t>();
        laserDataPointAmplitudeDifference = jConfig_.at(
          "LASER_DATAPOINT_AMPLITUDE_DIFFERENCE").get<float>();
        maxDistanceBetweenPts = jConfig_.at("MAX_DIST_BETWEEN_POINTS").get<uint8_t>();
    } catch (const nlohmann::json::exception& e) {
        logger_->warn("Failed to read config because: {}", e.what());
        return false;
    }
    return true;
}

}  // namespace personfollower
}  // namespace applications
}  // namespace crf
