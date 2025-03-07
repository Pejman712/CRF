/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <fcl/fcl.h>
#include <fcl/narrowphase/continuous_collision.h>

namespace crf::navigation::collisiondetector {

/*
 * Struct stores the collision request and the result given by collision algorithm
 * and the flag wether the collision iteration can stop
 */
struct FCLCollisionData {
    FCLCollisionData():
        request(),
        result(),
        done(false) {
    }
    fcl::CollisionRequest<double> request;
    fcl::CollisionResult<double> result;
    bool done;
};

/*
 * Struct stores the distance request and the result given by distance algorithm
 * and the flag wether the distance iteration can stop
 */
struct FCLDistanceData {
    FCLDistanceData():
        request(),
        result(),
        done(false) {
    }
    fcl::DistanceRequest<double> request;
    fcl::DistanceResult<double> result;
    bool done;
};

/*
 * Struct stores the continuous collision request and result given by the continuous collision algorithm
 * and the flag wether the continuous collision iteration can stop
 */

struct FCLContinuousCollisionData {
    FCLContinuousCollisionData():
        request(),
        result(),
        done(false) {
    }
    fcl::ContinuousCollisionRequest<double> request;
    fcl::ContinuousCollisionResult<double> result;
    bool done;
};

class FCLCollisionDetectorUtility {
 public:
    static bool defaultCollisionFunction(fcl::CollisionObject<double>* o1,
        fcl::CollisionObject<double>* o2, void* collisionData);
    static bool defaultDistanceFunction(fcl::CollisionObject<double>* o1,
        fcl::CollisionObject<double>* o2, void* distanceData, double& dist);  //NOLINT
};

}  // namespace crf::navigation::collisiondetector
