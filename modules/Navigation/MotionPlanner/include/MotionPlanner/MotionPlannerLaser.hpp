/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Julia Kabalar CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <octomap/OcTree.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "EventLogger/EventLogger.hpp"
#include "Laser/ILaser.hpp"
#include "CollisionDetector/ICollisionDetector.hpp"
#include "PathPlanner/IPathPlanner.hpp"
#include "RobotBase/RobotBaseConfiguration.hpp"
#include "RobotBaseController/IRobotBaseController.hpp"
#include "TrajectoryGenerator/ITaskTrajectoryGenerator.hpp"
#include "MotionPlanner/IMotionPlanner.hpp"
#include "Types/TaskTypes/TaskPose.hpp"

namespace crf {
namespace applications {
namespace motionplanner {

class MotionPlannerLaser : public IMotionPlanner {
 public:
    MotionPlannerLaser() = delete;
    MotionPlannerLaser(
        std::shared_ptr<crf::robots::robotbasecontroller::IRobotBaseController> baseController,
        std::shared_ptr<algorithms::pathplanner::IPathPlanner> planner,
        std::shared_ptr<algorithms::collisiondetector::ICollisionDetector> collisionChecker,
        std::shared_ptr<algorithms::
        trajectorygenerator::ITaskTrajectoryGenerator> trajectorygenerator,
        std::shared_ptr<robots::robotbase::RobotBaseConfiguration> robotConfig,
        std::shared_ptr<sensors::laser::ILaser> laser,
        const utility::types::TaskPose& laserPosition, float ocTreeResolution,
        bool saveData = false);

    // TODO(jukabala+adiazros): add position manager for laser/rgbd camera position, at the moment hardcoded NOLINT

    MotionPlannerLaser(const MotionPlannerLaser& other) = delete;
    MotionPlannerLaser(MotionPlannerLaser&& other) = delete;
    virtual ~MotionPlannerLaser();
    bool initialize() override;
    bool deinitialize() override;
    bool plan(const utility::types::TaskPose &goal) override;
    static std::vector<float> TaskPosePath2VectorPath(
        const crf::utility::types::TaskPose& pos);
    static crf::utility::types::TaskPose VectorPath2TaskPosePath(
        const std::vector<float> &path);

 private:
    utility::logger::EventLogger logger_;
    std::shared_ptr<crf::robots::robotbasecontroller::IRobotBaseController> baseController_;
    std::shared_ptr<algorithms::pathplanner::IPathPlanner> planner_;
    std::shared_ptr<algorithms::collisiondetector::ICollisionDetector> collisionChecker_;
    std::shared_ptr<algorithms::trajectorygenerator::ITaskTrajectoryGenerator> trajectorygenerator_;  // NOLINT
    const robots::robotbase::RobotParameters robotParameters_;
    std::shared_ptr<sensors::laser::ILaser> laser_;
    const utility::types::TaskPose laserPosition_;
    const float ocTreeResolution_;
    bool isInitialized_;
    utility::types::TaskPose robotPosition_;
    bool dataSavingEnabled_;
    bool updateMap();
    std::array<pcl::PointXYZRGBA, 2> createRobotBaseBox();
    octomap::OcTree createOctree(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&);
};

}  // namespace motionplanner
}  // namespace applications
}  // namespace crf
