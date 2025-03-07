#pragma once

/* Â© Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include "CommonInterfaces/IInitializable.hpp"
#include "Types/TaskTypes/TaskPose.hpp"

#include <pcl/common/centroid.h>

namespace crf {
namespace applications {
namespace personfollower {

struct PersonCentroid {
    PersonCentroid():
    objectID(-1),  // NOTE -1 necessary to check if returned struct is empty
    dissapeared(0),
    centroidPoint(),
    euclideanDistance() {
    }
    int objectID;
    int dissapeared;
    pcl::PointXYZRGBA centroidPoint;
    float euclideanDistance;
};

struct UnassignedCentroid {
    pcl::PointXYZRGBA centroidPoint;
    bool assigned;
};

struct ClusterPositionStruct {
    ClusterPositionStruct():
    startPoint(0),
    secondPoint(0),
    thirdPoint(0),
    endPoint(0) {
    }
    int startPoint;
    int secondPoint;
    int thirdPoint;
    int endPoint;
};

class IPersonTracker : public utility::commoninterfaces::IInitializable {
 public:
    virtual ~IPersonTracker() = default;
    bool initialize() override = 0;
    bool deinitialize() override = 0;

    virtual PersonCentroid trackPerson(const utility::types::TaskPose& cameraPose) = 0;
    virtual int getCameraDisplacement() = 0;
};

}  // namespace personfollower
}  // namespace applications
}  // namespace crf
