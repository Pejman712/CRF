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
#include <limits>

#include "EventLogger/EventLogger.hpp"
#include "RobotBase/RobotBaseConfiguration.hpp"
#include "CollisionDetector/FCLCollisionDetector/FCLCollisionDetectorUtility.hpp"
#include "CollisionDetector/FCLCollisionDetector/FCLCollisionDetector.hpp"

#define DEFAULT_ROBOT_HEIGHT 0.25
#define TASK_SPACE_SIZE 6

namespace crf::navigation::collisiondetector {

FCLCollisionDetector::FCLCollisionDetector(
    std::shared_ptr<actuators::robotbase::RobotBaseConfiguration> robotBaseConfig,
    std::vector<SpaceType> stateTypes,  float offsetFromWheelsDistance):
    pi_(static_cast<float>(M_PI)),
    logger_("FCLCollisionDetector"),
    robotBaseConfig_(robotBaseConfig),
    stateTypes_(stateTypes),
    robotGeometry_(
        new fcl::Box<double>(robotBaseConfig->getRobotParameters().wheelsDistanceY
        + offsetFromWheelsDistance, robotBaseConfig->getRobotParameters().wheelsDistanceX
        + offsetFromWheelsDistance, DEFAULT_ROBOT_HEIGHT)),
    treeGeometry_(),
    collisionManager_(new fcl::DynamicAABBTreeCollisionManager<double>()) {
    logger_->debug("CTor");
    if (robotBaseConfig_ == nullptr) {
        throw std::runtime_error("Invalid robot base configuration");
    }
    if (!checkStateSpaceDimensions()) {
        throw std::runtime_error("StateSpaceDimensions do not match TaskStateType");
    }
}

FCLCollisionDetector::~FCLCollisionDetector() {
    logger_->debug("DTor");
    clearCollisionManager();
}

bool FCLCollisionDetector::checkState(const std::vector<float> &state) {
    logger_->debug("checkState");
    if (!checkValidity(state)) {
        logger_->warn("Provided state is invalid");
        return false;
    }
    if (collisionManager_->empty()) {
        logger_->warn("Collision Objects have not been defined");
        return false;
    }
    fcl::CollisionObject<double> robot(robotGeometry_, getTransform(state));

    FCLCollisionData cdata;
    cdata.request.gjk_solver_type = fcl::GJKSolverType::GST_INDEP;

    collisionManager_->collide(&robot, &cdata,
                            FCLCollisionDetectorUtility::defaultCollisionFunction);

    if (cdata.result.isCollision()) {
        logger_->warn("Collision between robot and tree");
        return false;
    }
    return true;
}

bool FCLCollisionDetector::checkMotion(const std::vector<float> &initialState,
        const std::vector<float> &finalState) {
    logger_->debug("checkMotion");
    if ((!checkValidity(initialState)) || (!checkValidity(finalState))) {
        logger_->warn("Provided state is invalid");
        return false;
    }
    if (collisionManager_->empty()) {
        logger_->warn("Collision objects have not been defined");
        return false;
    }
    FCLContinuousCollisionData cdata;
    cdata.request.gjk_solver_type = fcl::GJKSolverType::GST_INDEP;

    // not using callback for continuous collision check
    fcl::continuousCollide(robotGeometry_.get(), getTransform(initialState),
                getTransform(finalState), treeGeometry_.get(), fcl::Transform3d::Identity(),
                fcl::Transform3d::Identity(), cdata.request, cdata.result);

    if (cdata.result.is_collide) {
        logger_->warn("Continuous collision detected time_of_contact {}",
                        cdata.result.time_of_contact);
        return false;
    }
    return true;
}

boost::optional<float> FCLCollisionDetector::clearance(const std::vector<float> &state) {
    logger_->debug("clearance");
    float distance = std::numeric_limits<float>::infinity();
    if (!checkValidity(state)) {
        logger_->warn("Provided state is invalid");
        return boost::none;
    }
    if (collisionManager_->empty()) {
        logger_->warn("Collision objects have not been defined");
        return boost::none;
    }
    fcl::CollisionObject<double> robot(robotGeometry_, getTransform(state));

    FCLDistanceData ddata;
    ddata.request.gjk_solver_type = fcl::GJKSolverType::GST_INDEP;

    collisionManager_->distance(&robot, &ddata,
                            FCLCollisionDetectorUtility::defaultDistanceFunction);

    if (ddata.result.min_distance < std::numeric_limits<float>::max()) {
        distance = static_cast<float>(ddata.result.min_distance);
        logger_->warn("Collision in {} distance", ddata.result.min_distance);
    }
    if (distance < 0) {
        distance = .0;  // Output distance when collision is -1, here setting it to zero
    }
    return distance;
}

bool FCLCollisionDetector::updateMap(const octomap::OcTree &tree) {
    logger_->debug("updateMap");
    if (!clearCollisionManager()) {
        logger_->warn("Could not clear collision manager");
        return false;
    }
    auto ocTree = std::make_shared<fcl::OcTree<double> >(
                  std::make_shared<const octomap::OcTree>(tree));
    treeGeometry_ = ocTree;

    std::vector<std::array<double, 6> > boxes_ = ocTree->toBoxes();
    if (boxes_.empty()) {
        logger_->warn("No ocTree collision objects have been registered");
        return false;
    }
    for (std::size_t i = 0; i < boxes_.size(); i++) {
        double x = boxes_[i][0];
        double y = boxes_[i][1];
        double z = boxes_[i][2];
        double size = boxes_[i][3];
        double cost = boxes_[i][4];
        double threshold = boxes_[i][5];
        fcl::Box<double> box(size, size, size);
        box.cost_density = cost;
        box.threshold_occupied = threshold;
        fcl::Transform3d trans = fcl::Transform3d::Identity();
        trans.translation() = fcl::Vector3d(x, y, z);
        collisionManager_->registerObject(new fcl::CollisionObject<double>(
                std::make_shared<fcl::Box<double>>(box), trans));
    }
    logger_->info("Registered {} collision objects", collisionManager_->size());
    collisionManager_->setup();
    return true;
}

fcl::Transform3d FCLCollisionDetector::getTransform(const std::vector<float> &state) {
    logger_->debug("getTransform");
    fcl::Transform3d trans = fcl::Transform3d::Identity();
    std::vector<double> receivedState;
    fcl::Vector3d receivedTranslation = fcl::Vector3d::Identity();
    for (std::size_t dimID = 0; dimID < state.size(); dimID++) {
        if (stateTypes_[dimID] == SpaceType::REALVECTOR_STATE_SPACE) {
            receivedState.push_back(state[dimID]);
        }
    }
    if (receivedState.size() > SE2_SPACE_SIZE) {
        return trans;
    }
    for (std::size_t i = 0; i < receivedState.size(); i++) {
        receivedTranslation(i) = receivedState[i];
    }
    trans.translation() = receivedTranslation;
    float receivedRotation2D = .0;
    for (std::size_t dimID = 0; dimID < state.size(); dimID++) {
        if (stateTypes_[dimID] == SpaceType::SO2_STATE_SPACE) {
            receivedRotation2D = state[dimID];
        }
    }
    fcl::Quaterniond rotation = fcl::AngleAxisd(.0, fcl::Vector3d::UnitX())
                              * fcl::AngleAxisd(.0, fcl::Vector3d::UnitY())
                              * fcl::AngleAxisd(receivedRotation2D, fcl::Vector3d::UnitZ());
    trans.linear() = rotation.toRotationMatrix();
    return trans;
}

bool FCLCollisionDetector::checkValidity(const std::vector<float> &state) {
    logger_->debug("checkValidity");
    if (state.size() != stateTypes_.size()) {
        logger_->error("The input state has a different dimension number than expected");
        return false;
    }
    for (std::size_t dimID = 0; dimID < state.size(); dimID++) {
        if (stateTypes_[dimID] == SpaceType::SO2_STATE_SPACE && std::abs(state[dimID]) > pi_) {
            logger_->error("The state is not within the SO(2) bounds");
            return false;
        }
    }
    return true;
}

bool FCLCollisionDetector::checkStateSpaceDimensions() {
    logger_->debug("checkStateSpaceDimensions");
    if (stateTypes_.size() > TASK_SPACE_SIZE || stateTypes_.empty()) {
        logger_->error("Number of dimensions doesn't match state types size");
        return false;
    }
    for (std::size_t dimID = 0; dimID < stateTypes_.size(); dimID++) {
        if (stateTypes_[dimID] != SpaceType::REALVECTOR_STATE_SPACE &&
            stateTypes_[dimID] != SpaceType::SO2_STATE_SPACE) {
            logger_->error("Invalid space type definition - Space not supported");
            return false;
        }
    }
    return true;
}

bool FCLCollisionDetector::clearCollisionManager() {
    logger_->debug("clearCollisionManager");
    std::vector<fcl::CollisionObject<double>*> managedObjects;
    collisionManager_->getObjects(managedObjects);
    for (std::size_t j = 0; j < managedObjects.size(); ++j) {
        delete managedObjects[j];
    }
    collisionManager_->clear();
    return collisionManager_->empty();
}

}  // namespace crf::navigation::collisiondetector
