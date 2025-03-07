/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include "CollisionDetector/FCLCollisionDetector/FCLCollisionDetectorUtility.hpp"

namespace crf::navigation::collisiondetector {

bool FCLCollisionDetectorUtility::defaultCollisionFunction(
    fcl::CollisionObject<double>* o1, fcl::CollisionObject<double>* o2, void* collisionData) {
    FCLCollisionData* cdata = static_cast<FCLCollisionData*>(collisionData);
    if (cdata->done) {
        return true;
    }
    fcl::collide(o1, o2, cdata->request, cdata->result);
    if (!cdata->request.enable_cost && (cdata->result.isCollision())
        && (cdata->result.numContacts() >= cdata->request.num_max_contacts)) {
        cdata->done = true;
    }
    return cdata->done;
}

bool FCLCollisionDetectorUtility::defaultDistanceFunction(fcl::CollisionObject<double>* o1,
        fcl::CollisionObject<double>* o2, void* distanceData, double& dist) {
    FCLDistanceData* ddata = static_cast<FCLDistanceData*>(distanceData);
    if (ddata->done) {
        return true;
    }
    fcl::distance(o1, o2, ddata->request, ddata->result);
    dist = ddata->result.min_distance;
    if (dist <= 0) {
        ddata->done = true;
        return true;
    }
    return ddata->done;
}

}  // namespace crf::navigation::collisiondetector
