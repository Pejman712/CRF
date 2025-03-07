/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Julia Kabalar CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

#include <string>
#include <memory>
#include <vector>
#include <limits>

#include <pcl/io/pcd_io.h>

// #include "RGBDVisionUtility/PCLUtils.hpp"
#include "MotionPlanner/MotionPlannerLaser.hpp"

#define RESOLUTION_OCTOMAP 0.005
#define DEFAULT_BOX_HEIGHT 0.5

namespace crf {
namespace applications {
namespace motionplanner {

MotionPlannerLaser::MotionPlannerLaser(
    std::shared_ptr<crf::robots::robotbasecontroller::IRobotBaseController> baseController,
    std::shared_ptr<algorithms::pathplanner::IPathPlanner> planner,
    std::shared_ptr<algorithms::collisiondetector::ICollisionDetector> collisionChecker,
    std::shared_ptr<
    algorithms::trajectorygenerator::ITaskTrajectoryGenerator> trajectorygenerator,
    std::shared_ptr<robots::robotbase::RobotBaseConfiguration> robotConfig,
    std::shared_ptr<sensors::laser::ILaser> laser,
    const utility::types::TaskPose& laserPosition,
    float ocTreeResolution = RESOLUTION_OCTOMAP, bool saveData):
    logger_("MotionPlannerLaser"),
    baseController_(baseController),
    planner_(planner),
    collisionChecker_(collisionChecker),
    trajectorygenerator_(trajectorygenerator),
    robotParameters_(robotConfig->getRobotParameters()),
    laser_(laser),
    laserPosition_(laserPosition),
    ocTreeResolution_(ocTreeResolution),
    isInitialized_(false),
    robotPosition_(),
    dataSavingEnabled_(saveData) {
    logger_->info("CTor");
}

MotionPlannerLaser::~MotionPlannerLaser() {
    logger_->debug("DTor");
    deinitialize();
}

bool MotionPlannerLaser::initialize() {
    logger_->debug("initialize");
    if (isInitialized_) {
        logger_->warn("Is already initialized");
        return false;
    }
    if (!laser_->initialize()) {
        logger_->warn("Could not initialize laser");
        return false;
    }
    if (!baseController_->initialize()) {
        logger_->warn("Could not initialize base controller");
        return false;
    }
    isInitialized_ = true;
    logger_->debug("initialized");
    return true;
}

bool MotionPlannerLaser::deinitialize() {
    logger_->debug("deinitialize");
    if (!isInitialized_) {
        logger_->warn("Could not deinitialize because not initialized");
        return false;
    }
    if (!laser_->deinitialize()) {
        logger_->warn("Could not deinitialize laser");
        return false;
    }
    baseController_->interruptTrajectory();
    if (!baseController_->deinitialize()) {
        logger_->warn("Could not deinitialize base controller");
        return false;
    }
    isInitialized_ = false;
    return true;
}

bool MotionPlannerLaser::updateMap() {
    logger_->info("updateMap");
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inPointCloud = laser_->getPointCloud();
    if (!inPointCloud) {
        logger_->error("Could not update point cloud");
        return false;
    }
    if (dataSavingEnabled_) {
        pcl::io::savePCDFileASCII("cloud.pcd", *inPointCloud);
    }
    robotPosition_ = baseController_->getPosition();

    // auto robotBox = createRobotBaseBox();
    auto newPCL = inPointCloud;  // utility::rgbdvisionutility::PCLUtils::extractBoxInliers(inPointCloud, robotBox); NOLINT
    auto ocTree = createOctree(newPCL);

    if (!collisionChecker_->updateMap(ocTree)) {
        logger_->error("Could not update collision checker");
        return false;
    }
    return true;
}

bool MotionPlannerLaser::plan(const utility::types::TaskPose &goal) {
    if (!updateMap()) {
        logger_->warn("Cannot ensure map has been updated");
        return false;
    }
    auto goalVector = TaskPosePath2VectorPath(goal);
    auto startVector = TaskPosePath2VectorPath(robotPosition_);
    if (!planner_->computePath(startVector, goalVector)) {
        logger_->error("Failed to plan path");
        return false;
    }

    auto pathLength = planner_->getPathLength();
    if (!pathLength) {
        logger_->error("Path is not of valid length");
        return false;
    }
    auto path = planner_->getPath();
    if (path.size() == 0) {
        logger_->error("Path is not of valid length");
        return false;
    }

    std::vector<utility::types::TaskPose> pathResult;
    for (auto& pathPoint : path) {
        if (pathPoint.size() != 3) {
            logger_->error("Did not plan with correct dimensions");
            return false;
        }
        pathResult.push_back(VectorPath2TaskPosePath(pathPoint));
    }
    if (dataSavingEnabled_) {
        // Save Data
        std::ofstream pathFile;
        pathFile.open("Path.csv");
        pathFile << "steps" << pathResult.size() << std::endl;
        for (size_t i = 0; i < pathResult.size(); i++) {
            pathFile
            << i << ";"
            << pathResult[i](0) << ";"
            << pathResult[i](1) << ";"
            << pathResult[i](2) << std::endl;
        }
        pathFile.close();
    }
/*
    if (!trajectorygenerator_->computeTrajectory(pathResult)) {
        logger_->error("Could not generate trajectory result");
        return false;
    }
*/
    std::future<bool> asynchrOperation = baseController_->setPosition(pathResult);

    if (!asynchrOperation.valid()) {
        logger_->error("Could not execute trajectory for path of size: {}", pathResult.size());
        return false;
    }

    return asynchrOperation.get();
}

std::array<pcl::PointXYZRGBA, 2> MotionPlannerLaser::createRobotBaseBox() {
    std::array<pcl::PointXYZRGBA, 2> robotBox;
    utility::types::TaskPose maxCornerRobot(
        {robotParameters_.wheelsDistanceX, robotParameters_.wheelsDistanceY, .0, .0, .0, .0});
    utility::types::TaskPose minCornerRobot(
        {-robotParameters_.wheelsDistanceX, -robotParameters_.wheelsDistanceY, .0, .0, .0, .0});
    auto transformedRobotPose = minCornerRobot * laserPosition_;
    transformedRobotPose = robotPosition_*transformedRobotPose;
    auto pointCloudGeneratorPose = transformedRobotPose.getPosRotMatrix();
    pcl::PointXYZRGBA referencePoint;
    referencePoint.x = pointCloudGeneratorPose[0];
    referencePoint.y = pointCloudGeneratorPose[1];
    referencePoint.z = pointCloudGeneratorPose[2]-DEFAULT_BOX_HEIGHT;

    robotBox[0] = referencePoint;
    transformedRobotPose = maxCornerRobot * laserPosition_;
    transformedRobotPose = robotPosition_*transformedRobotPose;
    pointCloudGeneratorPose = transformedRobotPose.getPosRotMatrix();
    referencePoint.x = pointCloudGeneratorPose[0];
    referencePoint.y = pointCloudGeneratorPose[1];
    referencePoint.z = pointCloudGeneratorPose[2]+DEFAULT_BOX_HEIGHT;

    robotBox[1] = referencePoint;
    return robotBox;
}


octomap::OcTree MotionPlannerLaser::createOctree
    (const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud) {
    logger_->debug("createOctree");
    octomap::Pointcloud octocloud;
    for (auto p : cloud->points)
        octocloud.push_back(p.x, p.y, p.z);

    octomap::OcTree ocTree(ocTreeResolution_);
    int maxRange = -1;
    Eigen::Vector3d cameraPosition = laserPosition_.getPosition();
    auto cameraCardanXYZ = laserPosition_.getCardanXYZ();

    ocTree.insertPointCloud(octocloud,
        octomap::point3d(cameraPosition(0), cameraPosition(1), cameraPosition(2)),
        octomap::pose6d(cameraPosition(0), cameraPosition(1), cameraPosition(2),
        cameraCardanXYZ[0], cameraCardanXYZ[1], cameraCardanXYZ[2]), maxRange, true, false);
    ocTree.updateInnerOccupancy();
    return ocTree;
}

crf::utility::types::TaskPose MotionPlannerLaser::VectorPath2TaskPosePath(
    const std::vector<float> &path) {
    std::vector<float> vec;
    for (size_t j = 0; j < path.size(); j++) {
        vec.push_back(path[j]);
    }
    crf::utility::types::TaskPose output({vec[0], vec[1], 0, 0, 0, vec[2]});
    return output;
}

std::vector<float> MotionPlannerLaser::TaskPosePath2VectorPath(
    const crf::utility::types::TaskPose& pos) {
    std::vector<float> output;
    output.push_back(pos(0));
    output.push_back(pos(1));
    output.push_back(pos(5));
    return output;
}

}  // namespace motionplanner
}  // namespace applications
}  // namespace crf
