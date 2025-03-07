/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

namespace crf::navigation::collisiondetector {

enum class MapRepresentation {
    NotDefined = 0,
    /*
     * 3D array of deterministic voxels (identified by their VoxelMap-like pointer address) that
     * hold a bitvector.
     */
    DeterministicVoxelMap = 1,
    /*
     * List of deterministic voxels (identified by their VoxelMap-like pointer address) that hold a
     * bitvector.
     */
    DeterministicVoxelList = 2,
    /*
     * Octree of deterministic voxels (identified by a Morton code) that hold a bitvector.
     */
    DeterministicOctree = 3,
    /*
     * List of deterministic voxels (identified by a Morton code) that hold a bitvector.
     */
    DeterministicMortonVoxelList = 4,
    /*
     * 3D array of probabilistic voxels (identified by their VoxelMap-like pointer address) that
     * hold a probability.
     */
    ProbabilisticVoxelMap = 5,
    /*
     * List of probabilistic voxels (identified by their VoxelMap-like pointer address) that hold a
     / probability.
     */
    ProbabilisticVoxelList = 6,
    /*
     * Octree of probabilistic voxels (identified by a Morton code) that hold a probability.
     */
    ProbabilisticOctree = 7,
    /*
     * List of probabilistic voxels (identified by a Morton code) that hold a probability.
     */
    ProbabilisticMortonVoxelList = 8,
};

}  // namespace crf::navigation::collisiondetector
