/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <vector>

#include <octomap/OcTree.h>

#include <boost/optional.hpp>

// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>

namespace crf::navigation::collisiondetector {

class ICollisionDetector {
 public:
    virtual ~ICollisionDetector() = default;
    /* 
     * Returns:
     *  - True if in that state there is no collision
     *  - False if there is any kind of collision when the body is in that position
     */
    virtual bool checkState(const std::vector<float> &state) = 0;
    /* 
     * Returns:
     *  - True if the motion of the body between the two states doesn't generate any collision
     *  - False if there is any kind of collision when the body moves from the initial state to
     *    the final state
     */
    virtual bool checkMotion(const std::vector<float> &initialState,
        const std::vector<float> &finalState) = 0;
    /* 
     * Returns:
     *  - True if octree has been successfully inserted into the modelled environment
     *  - False upon failure
     */
    virtual bool updateMap(const octomap::OcTree &tree) = 0;
    /*
     * Returns:
     *  - Value of smallest distance to the closest collision object in the robotic object's vicinity
     *  - Returns boost::none on empty map/failure
     */
    virtual boost::optional<float> clearance(const std::vector<float> &state) = 0;
    /* 
     * Returns:
     *  - True if previous objects in the map have been deleted and the point cloud has been successfully
     *    inserted into the modelled environment
     *  - False upon failure
     */
    // virtual bool updateMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) = 0;
};

}  // namespace crf::navigation::collisiondetector
