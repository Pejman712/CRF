#pragma once
/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <string>
#include <vector>
#include <memory>

#include <fcl/fcl.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/broadphase/broadphase_collision_manager.h>

#include "EventLogger/EventLogger.hpp"
#include "RobotBase/RobotBaseConfiguration.hpp"
#include "CollisionDetector/SpaceType.hpp"
#include "CollisionDetector/ICollisionDetector.hpp"

#define SE2_SPACE_SIZE 3

namespace crf::navigation::collisiondetector {

/*
 * This class implements a detector for possible collision between a robot and other objects in its vicinity
 * The state and motion checks can be used from path planning algorithms
 * The environment can be inserted in form of probabilistic octree
 */
class FCLCollisionDetector: public ICollisionDetector {
 public:
    FCLCollisionDetector(
        std::shared_ptr<actuators::robotbase::RobotBaseConfiguration> robotBaseConfig,
        std::vector<SpaceType> stateTypes, float offsetFromWheelsDistance = 0.25);
    ~FCLCollisionDetector() override;

    bool checkState(const std::vector<float> &state) override;
    bool checkMotion(const std::vector<float> &initialState,
        const std::vector<float> &finalState) override;
    bool updateMap(const octomap::OcTree &tree) override;
    boost::optional<float> clearance(const std::vector<float> &state) override;

 private:
    const float pi_;
    utility::logger::EventLogger logger_;
    std::shared_ptr<actuators::robotbase::RobotBaseConfiguration> robotBaseConfig_;
    std::vector<SpaceType> stateTypes_;
    std::shared_ptr<fcl::CollisionGeometry<double>> robotGeometry_;
    std::shared_ptr<fcl::CollisionGeometry<double>> treeGeometry_;
    std::shared_ptr<fcl::DynamicAABBTreeCollisionManager<double>> collisionManager_;
    /*
     * Returns:
     *  - transform in 3D for a collision object at a given state
     */
    fcl::Transform3d getTransform(const std::vector<float> &state);
    /* 
     * Returns:
     *  - True on a valid state i.e that the size of state must match the expected dimensions
     *    and given state values are within the expected bounds
     *  - False if validity criteria is not fulfilled
     */
    bool checkValidity(const std::vector<float> &state);
    /* 
     * Returns:
     *  - True if dimensions of state spaces match the dimensions for detection in 2D or 3D
     *  - False on invalidity
     */
    bool checkStateSpaceDimensions();
    /* 
     * Returns:
     *  - True if collision manager is cleared i.e. does not contain any collision objects
     *  - False if collision manager not empty
     */
    bool clearCollisionManager();
};

}  // namespace crf::navigation::collisiondetector
