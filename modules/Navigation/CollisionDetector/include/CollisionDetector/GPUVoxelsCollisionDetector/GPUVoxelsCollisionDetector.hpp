#pragma once
/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <string>
#include <vector>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/shared_ptr.hpp>
#include <gpu_voxels/GpuVoxels.h>
#include <map>

#include "EventLogger/EventLogger.hpp"
#include "CollisionDetector/ICollisionDetector.hpp"
#include "RobotArm/RobotArmConfiguration.hpp"
#include "CollisionDetector/SpaceType.hpp"
#include "CollisionDetector/MapRepresentation.hpp"

namespace crf::navigation::collisiondetector {

/*
 * Class that check if there is any collision between a robot and its environment and if the robot
 * collides with itself. This implementation is based in the library GPU Voxels. To construct an
 * object you need to pass the next arguments:
 *  - RobotArmConfiguration object that has the DH parameters of the robot.
 *  - The state types to know if we have a SO(2) space or a normal one.
 *  - The robot configuration file where the path of 3D models files, the home position, joints
 *    offset to adjust the representation to the real robot and the self-collision matrix is
 *    indicated. This matrix has a 0 if the collision between joints will be check and 1 for the
 *    opposite.
 *  - The detector configuration file defines if there is visualization, the size of the maps, the
 *    resolution of the movements in radians for the checkMotion method, the type of environment: 0
 *    for a predefined test environment, 1: to upload a file that is define with the path and a
 *    scale.
 * External documentation:
 *  - Web Page: https://www.gpu-voxels.org/
 *  - Publication: A. Hermann, F. Drews, J. Bauer, S. Klemm, A. Roennau and R. Dillmann: “Unified
 *    GPU Voxel Collision Detection for Mobile Manipulation Planning” in Intelligent Robots and
 *    Systems (IROS 2014, Chicago, September 2014)
 */
class GPUVoxelsCollisionDetector: public ICollisionDetector {
 public:
    GPUVoxelsCollisionDetector(
        std::shared_ptr<robots::robotarm::RobotArmConfiguration> robotArmConfig,
        std::vector<SpaceType> stateTypes,
        const std::string& robotConfigFile,
        const std::string& detectorConfigFile);
    ~GPUVoxelsCollisionDetector() override;

    bool checkState(const std::vector<float> &state) override;
    bool checkMotion(const std::vector<float> &initialState,
        const std::vector<float> &finalState) override;
    bool updateMap(const octomap::OcTree &tree) override;
    boost::optional<float> clearance(const std::vector<float> &state) override;
    bool updateMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);  // override

    // Methods use for debugging
    bool displayPath(const std::vector<std::vector<float>> &path);
    bool displayPerformanceResults();

 private:
    /*
     * Extraction of all the robot parameters defined in the configuration file to store as needed
     * for the other methods.
     * Returns:
     *  - True if it extract the parameters correctly.
     *  - False on failure.
     */
    bool setRobotConfiguration(const std::string &configFileName,
        std::shared_ptr<robots::robotarm::RobotArmConfiguration> robotArmConfig);
    /*
     * Extraction of all the collision detector and environment parameters defined in the
     * configuration file to store as needed for the other methods.
     * Returns:
     *  - True if it extract the parameters correctly.
     *  - False on failure.
     */
    bool setDetectorConfiguration(const std::string &configFileName);
    /*
     * Definition of the robot using the Denavit Hartenberg parameters into the GPU Voxels format.
     * The self-collision masks are also defined.
     * Returns:
     *  - True if the robot was added using the DH parameters.
     *  - False on failure.
     */
    bool setDenavitHartenbergParameters(const std::vector<robots::robotarm::DHParameter> &robotDH);
    /*
     * Creates a map depending of its type and reserves the amount of memory required for it.
     * Returns:
     *  - True if the environment representation is valid.
     *  - False on failure.
     */
    bool initializeMap(const std::string &mapName, const MapRepresentation &mapType);
    /*
     * The environment in which the robot is going to be is save and insert into the GPU Voxels map,
     * deleting all previous objects in the map.
     * Returns:
     *  - True if the environment type was set and inserted into the map.
     *  - False on failure.
     */
    bool insertEnvironment();
    /*
     * Determines the number of collisions between two maps, defined by its name and representation
     */
    boost::optional<std::size_t> getCollisions(const std::string &robotMapName,
        const MapRepresentation &robotMapType,
        const std::string &environmentMapName,
        const MapRepresentation &environmentMapType);
    /*
     * Determines the number of collisions between a deterministic VoxelMap and another map,
     * defined by its name and representation.
     */
    boost::optional<std::size_t> getRobotDeterministicVoxelMapCollisions(
        const std::string &robotMapName,
        const std::string &environmentMapName,
        const MapRepresentation &environmentMapType);
    /*
     * Determines the number of collisions between a deterministic Octree and another map,
     * defined by its name and representation.
     */
    boost::optional<std::size_t> getRobotDeterministicOctreeCollisions(
        const std::string &robotMapName,
        const std::string &environmentMapName,
        const MapRepresentation &environmentMapType);
    /*
     * Determines the number of collisions between a probabilistic VoxelMap and another map,
     * defined by its name and representation.
     */
    boost::optional<std::size_t> getRobotProbabilisticVoxelMapCollisions(
        const std::string &robotMapName,
        const std::string &environmentMapName,
        const MapRepresentation &environmentMapType);
    /*
     * Calculates the distance between to members depending on their space type. For real space the
     * distance is Euclidean and for the SO(2) spaces it takes into account angle-wrapping from -Pi
     * to Pi
     */
    boost::optional<float> getDistance(float initialValue, float finalValue, SpaceType type) const;
    /*
     * Get the number of segments between to states that ensure the minimum resolution en each
     * dimension
     */
    boost::optional<unsigned int> getNumberOfSegments(const std::vector<float> &initialState,
        const std::vector<float> &finalState,
        float resolution) const;
    /*
     * Computes the state that lies at the time t in [0, 1] on the segment that connects from the
     * initial state to the final state.
     */
    std::vector<float> interpolateStates(const std::vector<float> &initialState,
        const std::vector<float> &finalState,
        float time) const;
    /*
     * Computes the value that lies at the time t in [0, 1] on the segment that connects from the
     * initial value to the final value in a real space.
     */
    float interpolateRealSpaceValues(float initialValue,
        float finalValue,
        float time) const;
    /*
     * Computes the value that lies at the time t in [0, 1] on the segment that connects from the
     * initial value to the final value in a real space.
     */
    float interpolateSO2SpaceValues(float initialValue,
        float finalValue,
        float time) const;

    const float pi_;
    const float pihalf_;
    const float twopi_;

    utility::logger::EventLogger logger_;
    std::vector<SpaceType> stateTypes_;
    boost::shared_ptr<gpu_voxels::GpuVoxels> gpuVoxels_;
    std::vector<gpu_voxels::BitVoxelMeaning> voxelMeanings_;
    std::vector<gpu_voxels::BitVector<gpu_voxels::BIT_VECTOR_LENGTH>> collisionMasks_;
    unsigned int jointsNumber_;
    unsigned int linksNumber_;
    std::string linksDirectory_;
    std::vector<std::string> linkNames_;
    std::vector<std::string> linkFileNames_;
    std::vector<float> homeState_;
    std::vector<float> stateOffset_;
    std::vector<std::vector<int>> selfCollisionMatrix_;
    bool visualization_;
    std::uint32_t spaceVolume_[3];
    float spaceResolution_;
    float centerPosition_[3];
    float motionResolution_;
    MapRepresentation robotMapRepresentation_;
    std::string environmentFile_;
    unsigned int environmentType_;
    float environmentScale_;
    std::vector<std::array<float, 6>> cubes_;
    MapRepresentation environmentMapRepresentation_;
    std::map<std::string, MapRepresentation> mapRepresentationTypeInterpreter;
};

}  // namespace crf::navigation::collisiondetector
