/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

// Required for the use of the PerformanceMonitor class, a tool for performance measurement.
#define IC_PERFORMANCE_MONITOR

#include <string>
#include <vector>
#include <memory>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <nlohmann/json.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <experimental/filesystem>
#include <gpu_voxels/GpuVoxels.h>
#include <gpu_voxels/logging/logging_gpu_voxels.h>
#include <icl_core_performance_monitor/PerformanceMonitor.h>
#include <boost/assign/list_inserter.hpp>

#include "EventLogger/EventLogger.hpp"
#include "CollisionDetector/GPUVoxelsCollisionDetector/GPUVoxelsCollisionDetector.hpp"
#include "RobotArm/RobotArmConfiguration.hpp"
#include "CollisionDetector/SpaceType.hpp"
#include "CollisionDetector/MapRepresentation.hpp"

// The default value of the collision detection threshold of the method collideWith is 1.0f, which
// would register probabilistic voxels with maximum occupancy probability as occupied. This is a
// bad assumption if the map is a probabilistic VoxelMap, where the obstacles have a occupancy of
// about 80%.
#define OCCUPANCY_THRESHOLD_FOR_COLLISIONS_IN_PROBABILISTIC_MAPS 0.1f

namespace crf::navigation::collisiondetector {

GPUVoxelsCollisionDetector::GPUVoxelsCollisionDetector(
    std::shared_ptr<robots::robotarm::RobotArmConfiguration> robotArmConfig,
    std::vector<SpaceType> stateTypes,
    const std::string& robotConfigFile,
    const std::string& detectorConfigFile) :
    pi_(static_cast<float>(M_PI)),
    pihalf_(static_cast<float>(M_PI/2.0)),
    twopi_(static_cast<float>(M_PI*2.0)),
    logger_("GPUVoxelsCollisionDetector"),
    stateTypes_(stateTypes) {
    logger_->debug("CTor");
    icl_core::logging::initialize();
    // Initializes the performance monitor with an approximate number of names and events. These
    // numbers don't have to be exact, they only define the size of the underlying data structures.
    PERF_MON_INITIALIZE(100, 1000);  // (Number of Names, Number of Events)

    boost::assign::insert(mapRepresentationTypeInterpreter)
        ("Deterministic VoxelMap", MapRepresentation::DeterministicVoxelMap)
        ("Deterministic VoxelList", MapRepresentation::DeterministicVoxelList)
        ("Deterministic Octree", MapRepresentation::DeterministicOctree)
        ("Deterministic Morton VoxelList", MapRepresentation::DeterministicMortonVoxelList)
        ("Probabilistic VoxelMap", MapRepresentation::ProbabilisticVoxelMap)
        ("Probabilistic VoxelList", MapRepresentation::ProbabilisticVoxelList)
        ("Probabilistic Octree", MapRepresentation::ProbabilisticOctree)
        ("Probabilistic Morton VoxelList", MapRepresentation::ProbabilisticMortonVoxelList);

    if (robotArmConfig == nullptr) {
        throw std::runtime_error("Invalid robot arm configuration");
    }
    if (!setRobotConfiguration(robotConfigFile, robotArmConfig)) {
        throw std::runtime_error("Failed to extract the robot configuration parameters");
    }
    if (!setDetectorConfiguration(detectorConfigFile)) {
        throw std::runtime_error("Failed to extract the detector configuration parameters");
    }
    if (stateTypes_.size() != jointsNumber_) {
        throw std::runtime_error("Number of joints doesn't match state types size");
    }
    for (std::size_t jointID = 0; jointID < jointsNumber_; jointID++) {
        if (stateTypes_[jointID] != SpaceType::REALVECTOR_STATE_SPACE &&
            stateTypes_[jointID] != SpaceType::SO2_STATE_SPACE) {
            throw std::runtime_error("Invalid space type definition - Space not supported");
        }
    }
    // We generate an API class, which defines the volume of our space and the resolution. Be
    // careful here! The size is limited by the memory of your GPU. Even if an empty Octree is
    // small, a Voxelmap will always require full memory.
    gpuVoxels_ = gpu_voxels::GpuVoxels::getInstance();
    gpuVoxels_->initialize(spaceVolume_[0], spaceVolume_[1], spaceVolume_[2], spaceResolution_);

    // We add different maps with objects, to collide them
    gpuVoxels_->addMap(MT_BITVECTOR_VOXELMAP, "RobotSelfMap");
    if (!initializeMap("RobotMap", robotMapRepresentation_)) {
        throw std::runtime_error("Failed to create the robot map");
    }
    if (!initializeMap("EnvironmentMap", environmentMapRepresentation_)) {
        throw std::runtime_error("Failed to create the environment map");
    }

    // Add the maps to the visualizer.
    if (visualization_) {
        gpuVoxels_->addMap(MT_BITVECTOR_VOXELLIST, "SolutionMap");
        gpuVoxels_->visualizeMap("RobotMap");
        gpuVoxels_->visualizeMap("EnvironmentMap");
        gpuVoxels_->visualizeMap("SolutionMap");
    }

    // Enables the timer under the given prefix.
    PERF_MON_ENABLE("RobotStateChecker");
    PERF_MON_ENABLE("RobotMotionChecker");

    if (!setDenavitHartenbergParameters(robotArmConfig->getKinematicChain())) {
        throw std::runtime_error("Failed to configure the robot");
    }
    if (!insertEnvironment()) {
        throw std::runtime_error("Failed to insert environment");
    }
}

GPUVoxelsCollisionDetector::~GPUVoxelsCollisionDetector() {
    logger_->debug("DTor");
}

bool GPUVoxelsCollisionDetector::checkState(const std::vector<float> &state) {
    logger_->debug("checkState");
    if (state.size() != jointsNumber_) {
        logger_->error("The input state has a different dimension number than a predefined one");
        return false;
    }
    // Save the values of the state and pass them to the robot articulations format
    gpu_voxels::robot::JointValueMap stateJointValues;
    for (std::size_t jointID = 0; jointID < jointsNumber_; jointID++) {
        if (std::abs(state[jointID]) > pi_ &&
            stateTypes_[jointID] == SpaceType::SO2_STATE_SPACE) {
            logger_->error("The state is not within the SO(2) bounds");
            return false;
        }
        stateJointValues[linkNames_[jointID]] = state[jointID] + stateOffset_[jointID];
    }
    gpuVoxels_->setRobotConfiguration("Robot", stateJointValues);

    // Calculate the number of self-collision of the Robot
    PERF_MON_START("SelfCollisionDetector");  // Start a timer with the given identifier.
    gpuVoxels_->clearMap("RobotSelfMap");
    gpu_voxels::BitVector<BIT_VECTOR_LENGTH> collidingMeanings;
    if (gpuVoxels_->insertRobotIntoMapSelfCollAware("Robot",
        "RobotSelfMap",
        voxelMeanings_,
        collisionMasks_,
        &collidingMeanings)) {
        logger_->info("State validation - Self-collisions detected");
        return false;
        }
    logger_->info("State validation - No self-collisions detected");
    PERF_MON_SILENT_MEASURE_AND_RESET_INFO_P("SelfCollisionDetector",
        "Self-collision detector of the Robot",
        "RobotStateChecker");  // Stop the timer and save

    PERF_MON_START("RobotInsertion");  // Start a timer with the given identifier.
    gpuVoxels_->clearMap("RobotMap");
    gpuVoxels_->insertRobotIntoMap("Robot", "RobotMap", gpu_voxels::eBVM_OCCUPIED);
    PERF_MON_SILENT_MEASURE_AND_RESET_INFO_P("RobotInsertion",
        "State insertion of the robot in the map",
        "RobotStateChecker");  // Stop the timer and save

    // Calculate the number of collisions between the Robot state and the environment
    PERF_MON_START("CollisionDetector");  // Start a timer with the given identifier.
    auto collisionsNumber = getCollisions("RobotMap",
        robotMapRepresentation_,
        "EnvironmentMap",
        environmentMapRepresentation_);
    if (!collisionsNumber) {
        logger_->error("The number of collisions can not be determined");
        return false;
    }
    PERF_MON_SILENT_MEASURE_AND_RESET_INFO_P("CollisionDetector",
        "Collision detector between the Robot state and the environment",
        "RobotStateChecker");  // Stop the timer and save

    // Add arbitrary floating point data to the statistics.
    PERF_MON_ADD_DATA_NONTIME_P("Number of collisions in a single state",
        static_cast<double>(collisionsNumber.get()),
        "RobotStateChecker");

    if (visualization_) {
        gpuVoxels_->visualizeMap("RobotMap");
        gpuVoxels_->visualizeMap("EnvironmentMap");
    }

    logger_->info("State validation - {} Collisions detected", collisionsNumber.get());
    return collisionsNumber.get() == 0;
}

bool GPUVoxelsCollisionDetector::checkMotion(const std::vector<float> &initialState,
    const std::vector<float> &finalState) {
    logger_->debug("checkMotion");
    if (initialState.size() != jointsNumber_ || finalState.size() != jointsNumber_) {
        logger_->error("The input states have a different dimension number than a predefined one");
        return false;
    }
    auto segmentsHolder = getNumberOfSegments(initialState, finalState, motionResolution_);
    if (!segmentsHolder) {
        logger_->error("Failed to get the number of segments of the motion");
        return false;
    }
    unsigned int segments = segmentsHolder.get();
    // Add arbitrary floating point data to the statistics.
    PERF_MON_ADD_DATA_NONTIME_P("Number of states in the motion", segments, "RobotMotionChecker");

    PERF_MON_START("RobotInsertion");  // Start a timer with the given identifier.
    gpuVoxels_->clearMap("RobotMap");
    for (std::size_t i = 0; i <= segments; i++) {
        // Save the values of the current state and pass them to the robot articulations format
        std::vector<float> currentState;
        gpu_voxels::robot::JointValueMap stateJointValues;
        currentState = interpolateStates(initialState,
            finalState,
            static_cast<float>(i)/static_cast<float>(segments));
        if (currentState.empty()) {
            logger_->error("Unable to interpolate between the two states");
            return false;
        }
        for (std::size_t jointID = 0; jointID < jointsNumber_; jointID++) {
            stateJointValues[linkNames_[jointID]] = currentState[jointID] + stateOffset_[jointID];
        }
        gpuVoxels_->setRobotConfiguration("Robot", stateJointValues);

        // Calculate the number of self collision of the Robot
        PERF_MON_START("SelfCollisionDetector");  // Start a timer with the given identifier.
        gpuVoxels_->clearMap("RobotSelfMap");
        gpu_voxels::BitVector<BIT_VECTOR_LENGTH> collidingMeanings;
        if (gpuVoxels_->insertRobotIntoMapSelfCollAware("Robot",
            "RobotSelfMap",
            voxelMeanings_,
            collisionMasks_,
            &collidingMeanings)) {
            logger_->info("Motion validation - Self-collisions detected");
            return false;
        }
        logger_->info("Motion validation - No self-collisions detected");
        PERF_MON_SILENT_MEASURE_AND_RESET_INFO_P("SelfCollisionDetector",
            "Self-collision detector of the Robot",
            "RobotMotionChecker");  // Stop the timer and save

        PERF_MON_START("RobotInsertion");  // Start a timer with the given identifier.
        gpuVoxels_->insertRobotIntoMap("Robot", "RobotMap", gpu_voxels::eBVM_OCCUPIED);
        PERF_MON_SILENT_MEASURE_AND_RESET_INFO_P("RobotInsertion",
            "State insertion of the robot in the map",
            "RobotMotionChecker");  // Stop the timer and save
    }

    // Calculate the number of collision between the Robot state and the environment
    PERF_MON_START("CollisionDetector");  // Start a timer with the given identifier.
    auto collisionsNumber = getCollisions("RobotMap",
        robotMapRepresentation_,
        "EnvironmentMap",
        environmentMapRepresentation_);
    if (!collisionsNumber) {
        logger_->error("The number of collisions can not be determined");
        return false;
    }
    PERF_MON_SILENT_MEASURE_AND_RESET_INFO_P("CollisionDetector",
        "Collision detector between the Robot motion and the environment",
        "RobotMotionChecker");  // Stop the timer and save

    // Add arbitrary floating point data to the statistics.
    PERF_MON_ADD_DATA_NONTIME_P("Number of collisions in a motion",
        static_cast<double>(collisionsNumber.get()),
        "RobotStateChecker");

    if (visualization_) {
        gpuVoxels_->visualizeMap("RobotMap");
        gpuVoxels_->visualizeMap("EnvironmentMap");
    }

    logger_->info("Motion validation - {} Collisions detected - {} States in the motion",
        collisionsNumber.get(),
        segments);
    return collisionsNumber.get() == 0;
}

bool GPUVoxelsCollisionDetector::updateMap(const octomap::OcTree &tree) {
    logger_->debug("updateMap");
    logger_->warn("Not implemented");
    return false;
}

boost::optional<float> GPUVoxelsCollisionDetector::clearance(const std::vector<float> &state) {
    logger_->debug("clearance");
    logger_->warn("Not implemented");
    return boost::none;
}

bool GPUVoxelsCollisionDetector::updateMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    logger_->debug("updateMap");
    gpuVoxels_->clearMap("EnvironmentMap");
    if (environmentType_ != 1 && environmentType_ != 2) {
        logger_->error("The environment type is incompatible with this method");
        return false;
    }
    std::vector<gpu_voxels::Vector3f> pointCloudVector;
    for (pcl::PointCloud<pcl::PointXYZ>::iterator i = cloud->begin(); i != cloud->end(); i++) {
        gpu_voxels::Vector3f point(
            (i->x*environmentScale_) + centerPosition_[0],
            (i->y*environmentScale_) + centerPosition_[1],
            (i->z*environmentScale_) + centerPosition_[2]);
        pointCloudVector.push_back(point);
    }
    bool result = gpuVoxels_->insertPointCloudIntoMap(pointCloudVector,
        "EnvironmentMap",
        gpu_voxels::eBVM_OCCUPIED);
    if (visualization_) {
        gpuVoxels_->visualizeMap("EnvironmentMap");
    }
    return result;
}

//                                    METHODS USE FOR DEBUGGING
// ================================================================================================

bool GPUVoxelsCollisionDetector::displayPath(const std::vector<std::vector<float>> &path) {
    logger_->debug("displayPath");
    if (!visualization_) {
        logger_->error("Visualization Disabled");
        return false;
    }
    gpuVoxels_->clearMap("RobotMap");
    gpuVoxels_->clearMap("SolutionMap");
    // Loop to add all the poses of the path to the swept volume
    gpu_voxels::robot::JointValueMap stateJointValues;
    for (std::size_t step = 0; step < path.size(); step++) {
        for (std::size_t jointID = 0; jointID < jointsNumber_; jointID++) {
            stateJointValues[linkNames_[jointID]] = path[step][jointID] + stateOffset_[jointID];
        }
        gpuVoxels_->setRobotConfiguration("Robot", stateJointValues);
        gpuVoxels_->insertRobotIntoMap("Robot", "SolutionMap",
            BitVoxelMeaning(eBVM_SWEPT_VOLUME_START + (step%249)));

        // Display information about the percentage left - Bar and Number
        std::string barPos(static_cast<int>((step*100)/(path.size()*2)), 35);
        std::string barNeg(50-static_cast<int>((step*100)/(path.size()*2)), 95);
        logger_->info("{}{} - {}%", barPos, barNeg, static_cast<int>((step*100)/path.size()));
    }
    gpuVoxels_->visualizeMap("SolutionMap");
    logger_->info("################################################## - 100%");
    return true;
}

bool GPUVoxelsCollisionDetector::displayPerformanceResults() {
    logger_->debug("visualizeSolution");
    // Print summary of all events from a given prefix
    PERF_MON_SUMMARY_PREFIX_INFO("RobotStateChecker");
    PERF_MON_SUMMARY_PREFIX_INFO("RobotMotionChecker");
    logger_->info("Robot consists of {} points",
        gpuVoxels_->getRobot("Robot")->getTransformedClouds()->getAccumulatedPointcloudSize());
    return true;
}

// ================================================================================================

bool GPUVoxelsCollisionDetector::setRobotConfiguration(const std::string &configFileName,
    std::shared_ptr<robots::robotarm::RobotArmConfiguration> robotArmConfig) {
    logger_->debug("setRobotConfiguration");
    std::ifstream configFile(configFileName);
    logger_->info("Opening {}", configFileName);
    if (!configFile.is_open()) {
        logger_->error("Failed to open configuration file: {}", configFileName);
        return false;
    }
    nlohmann::json jsonConfig;
    try {
        logger_->info("Loading JSON", configFileName);
        configFile >> jsonConfig;
    } catch (const std::exception& e) {
        logger_->error("JSON parse error: {}", e.what());
        return false;
    }
    configFile.close();
    try {
        jointsNumber_ = robotArmConfig->getNumberOfJoints();
        linksNumber_ = jointsNumber_ + 1;
        linksDirectory_ = __FILE__;
        linksDirectory_ = linksDirectory_.substr(0, linksDirectory_.find("cpproboticframework"));
        linksDirectory_ += jsonConfig.at("LinksDirectory").get<std::string>();

        std::vector<std::vector<int>> tempMatrix(linksNumber_, std::vector<int>(linksNumber_));
        for (std::size_t i = 0; i < linksNumber_; i++) {
            std::string ID = "Link" + std::to_string(i);
            linkFileNames_.push_back(jsonConfig.at("LinkFileNames").at(ID).get<std::string>());
            for (std::size_t j = 0; j < linksNumber_; j++) {
                tempMatrix[i][j] = jsonConfig.at("SelfCollisionMatrix").at(ID)[j].get<int>();
            }
        }
        selfCollisionMatrix_ = tempMatrix;

        for (std::size_t i = 1; i <= jointsNumber_; i++) {
            std::string jointID = "Joint" + std::to_string(i);
            homeState_.push_back(jsonConfig.at("HomeState").at(jointID).get<float>());
            stateOffset_.push_back(jsonConfig.at("StateOffset").at(jointID).get<float>());
        }
    } catch (const std::exception& e) {
        logger_->error("Failed to read the parameters: {}", e.what());
        return false;
    }
    return true;
}

bool GPUVoxelsCollisionDetector::setDetectorConfiguration(const std::string &configFileName) {
    logger_->debug("setDetectorConfiguration");
    std::ifstream configFile(configFileName);
    logger_->info("Opening {}", configFileName);
    if (!configFile.is_open()) {
        logger_->error("Failed to open configuration file: {}", configFileName);
        return false;
    }
    nlohmann::json jsonConfig;
    try {
        logger_->info("Loading JSON", configFileName);
        configFile >> jsonConfig;
    } catch (const std::exception& e) {
        logger_->error("JSON parse error: {}", e.what());
        return false;
    }
    configFile.close();
    try {
        visualization_ = jsonConfig.at("VisualizeSolution").get<bool>();
        spaceVolume_[0] = jsonConfig.at("Space").at("Volume").at("X").get<std::uint32_t>();
        spaceVolume_[1] = jsonConfig.at("Space").at("Volume").at("Y").get<std::uint32_t>();
        spaceVolume_[2] = jsonConfig.at("Space").at("Volume").at("Z").get<std::uint32_t>();
        spaceResolution_ = jsonConfig.at("Space").at("Resolution").get<float>();
        centerPosition_[0] = jsonConfig.at("CenterPosition").at("X").get<float>();
        centerPosition_[1] = jsonConfig.at("CenterPosition").at("Y").get<float>();
        centerPosition_[2] = jsonConfig.at("CenterPosition").at("Z").get<float>();
        motionResolution_ = jsonConfig.at("Robot").at("MotionResolution").get<float>();
        robotMapRepresentation_ = mapRepresentationTypeInterpreter[
            jsonConfig.at("Robot").at("Representation").get<std::string>()];
        environmentType_ = jsonConfig.at("Environment").at("Type").get<unsigned int>();
        if (environmentType_ == 1) {
            environmentScale_ = jsonConfig.at("Environment").at("Scale").get<float>();
        } else if (environmentType_ == 2) {
            environmentFile_ = __FILE__;
            environmentFile_ = environmentFile_.substr(0,
                environmentFile_.find("CERNRoboticFramework"));
            environmentFile_ += jsonConfig.at("Environment").at("File").get<std::string>();
            environmentScale_ = jsonConfig.at("Environment").at("Scale").get<float>();
        } else if (environmentType_ == 3) {
            unsigned int cubesNum = jsonConfig.at("Environment")
                .at("CubesNumber").get<unsigned int>();
            cubes_.resize(cubesNum);
            for (unsigned int i = 0; i < cubesNum; i++) {
                std::string cubeName = "Cube" + std::to_string(i+1);
                cubes_[i][0] = jsonConfig.at("Environment").at(cubeName)
                    .at("LowerCorner")[0].get<float>();
                cubes_[i][1] = jsonConfig.at("Environment").at(cubeName)
                    .at("LowerCorner")[1].get<float>();
                cubes_[i][2] = jsonConfig.at("Environment").at(cubeName)
                    .at("LowerCorner")[2].get<float>();
                cubes_[i][3] = jsonConfig.at("Environment").at(cubeName)
                    .at("HigherCorner")[0].get<float>();
                cubes_[i][4] = jsonConfig.at("Environment").at(cubeName)
                    .at("HigherCorner")[1].get<float>();
                cubes_[i][5] = jsonConfig.at("Environment").at(cubeName)
                    .at("HigherCorner")[2].get<float>();
            }
        } else {
            logger_->error("Unknown environment type in the configuration file: {}",
                configFileName);
            return false;
        }
        environmentMapRepresentation_ = mapRepresentationTypeInterpreter[
            jsonConfig.at("Environment").at("Representation").get<std::string>()];
    } catch (const std::exception& e) {
        logger_->error("Failed to read the parameters: {}", e.what());
        return false;
    }
    if (environmentType_ != 2) {
        return true;
    }
    std::ifstream file(environmentFile_);
    if (!file.good()) {
        logger_->error("The file {} does not exist", environmentFile_);
        return false;
    }
    return true;
}

bool GPUVoxelsCollisionDetector::setDenavitHartenbergParameters(
    const std::vector<robots::robotarm::DHParameter> &robotDH) {
    logger_->debug("setDenavitHartenbergParameters");

    linkNames_.resize(linksNumber_);
    unsigned int extraForTranslation = 3;
    std::vector<std::string> modelsPath(linksNumber_);
    std::vector<std::string> linkNamesWithExtra(linksNumber_ + extraForTranslation);

    // Define the names and models path of the links. The first links are created to move the robot
    // through the space. They don't affect the defined problem.
    linkNamesWithExtra[0] = "ZTranslation";
    linkNamesWithExtra[1] = "XTranslation";
    linkNamesWithExtra[2] = "YTranslation";
    for (std::size_t linkID = 0; linkID < linksNumber_; linkID++) {
        linkNames_[linkID] = linksDirectory_ + linkFileNames_[linkID];
        modelsPath[linkID] = linkNames_[linkID];
        linkNamesWithExtra[linkID + extraForTranslation] = linkNames_[linkID];
        std::ifstream file(modelsPath[linkID]);
        if (!file.good()) {
            logger_->error("The file {} does not exist", modelsPath[linkID]);
            return false;
        }
    }

    // Set the Denavit Hartenberg parameter (D, Theta, A, Alpha)
    std::vector<gpu_voxels::robot::DHParameters> parametersDH(linksNumber_ + extraForTranslation);
    parametersDH[0] = robot::DHParameters(0.0f, 0.0f, 0.0f, -pihalf_,
        centerPosition_[2], gpu_voxels::robot::PRISMATIC);  // Z Translation
    parametersDH[1] = robot::DHParameters(0.0f, -pihalf_, 0.0f, -pihalf_,
        centerPosition_[1], gpu_voxels::robot::PRISMATIC);  // Y Translation
    parametersDH[2] = robot::DHParameters(0.0f, pihalf_, 0.0f, pihalf_,
        centerPosition_[0], gpu_voxels::robot::PRISMATIC);  // X Translation
    for (std::size_t jointID = 0; jointID < jointsNumber_; jointID++) {
        if (robotDH[jointID].type == robots::robotarm::DHParameter::JointType::Rotational) {
            parametersDH[jointID + extraForTranslation] = robot::DHParameters(
                robotDH[jointID].d,
                robotDH[jointID].theta,
                robotDH[jointID].a,
                robotDH[jointID].alpha,
                homeState_[jointID]+stateOffset_[jointID],
                gpu_voxels::robot::REVOLUTE);
        } else if (robotDH[jointID].type == robots::robotarm::DHParameter::JointType::Linear) {
            if (stateTypes_[jointID] != SpaceType::REALVECTOR_STATE_SPACE) {
                logger_->error("The linear joints can only be in a real vector space");
                return false;
            }
            parametersDH[jointID + extraForTranslation] = robot::DHParameters(
                robotDH[jointID].d,
                robotDH[jointID].theta,
                robotDH[jointID].a,
                robotDH[jointID].alpha,
                homeState_[jointID]+stateOffset_[jointID],
                gpu_voxels::robot::PRISMATIC);
        } else {
            logger_->error("The type of the joint type is not correctly defined");
            return false;
        }
    }
    if (!gpuVoxels_->addRobot("Robot", linkNamesWithExtra, parametersDH, modelsPath, false)) {
        logger_->error("Failed to add the robot using the Denavit Hartenberg parameters");
        return false;
    }

    // Set the links that will be check for self-collision
    voxelMeanings_.resize(linksNumber_);
    collisionMasks_.resize(linksNumber_);
    gpu_voxels::BitVector<BIT_VECTOR_LENGTH> noCollisionMask;
    gpu_voxels::BitVector<BIT_VECTOR_LENGTH> allCollisionMask = ~noCollisionMask;  // Set all true
    allCollisionMask.clearBit(eBVM_COLLISION);

    for (std::size_t i = 0; i < linksNumber_; ++i) {
        voxelMeanings_[i] = BitVoxelMeaning(extraForTranslation + i);
    }
    for (std::size_t i = 0; i < linksNumber_; i++) {
        collisionMasks_[i] = allCollisionMask;
        for (std::uint32_t j = 0; j < linksNumber_; j++) {
            if (selfCollisionMatrix_[i][j] == 0) {
                collisionMasks_[i].clearBit(extraForTranslation + j);
            }
        }
    }
    return true;
}

bool GPUVoxelsCollisionDetector::initializeMap(const std::string &mapName,
    const MapRepresentation &mapType) {
    logger_->debug("initializeMaps");
    switch (mapType) {
        case MapRepresentation::DeterministicVoxelMap: {
            gpuVoxels_->addMap(MT_BITVECTOR_VOXELMAP, mapName);
            break;
        }
        case MapRepresentation::DeterministicOctree: {
            gpuVoxels_->addMap(MT_BITVECTOR_OCTREE, mapName);
            break;
        }
        case MapRepresentation::ProbabilisticVoxelMap: {
            gpuVoxels_->addMap(MT_PROBAB_VOXELMAP, mapName);
            break;
        }
        default: {
            logger_->error("The map representation is not implemented");
            return false;
            break;
        }
    }
    return true;
}

bool GPUVoxelsCollisionDetector::insertEnvironment() {
    logger_->debug("insertEnvironment");
    gpuVoxels_->clearMap("EnvironmentMap");
    switch (environmentType_) {
        case 2: {
            bool result = gpuVoxels_->insertPointCloudFromFile(
                "EnvironmentMap",
                environmentFile_,
                false,
                gpu_voxels::eBVM_OCCUPIED,
                false,
                Vector3f(centerPosition_[0], centerPosition_[1], centerPosition_[2]),
                environmentScale_);
            if (!result) {
                logger_->error("Failed to insert the Point Cloud");
                return false;
            }
            break;
        }
        case 3: {
            for (unsigned int i = 0; i < cubes_.size(); i++) {
                gpuVoxels_->insertBoxIntoMap(
                    gpu_voxels::Vector3f(
                        centerPosition_[0] + cubes_[i][0],
                        centerPosition_[1] + cubes_[i][1],
                        centerPosition_[2] + cubes_[i][2]),
                    gpu_voxels::Vector3f(
                        centerPosition_[0] + cubes_[i][3],
                        centerPosition_[1] + cubes_[i][4],
                        centerPosition_[2] + cubes_[i][5]),
                    "EnvironmentMap", gpu_voxels::eBVM_OCCUPIED, 2);
            }
        }
    }
    if (visualization_) {
        gpuVoxels_->visualizeMap("EnvironmentMap");
    }
    return true;
}

boost::optional<std::size_t> GPUVoxelsCollisionDetector::getCollisions(
    const std::string &robotMapName,
    const MapRepresentation &robotMapType,
    const std::string &environmentMapName,
    const MapRepresentation &environmentMapType) {
    logger_->debug("getCollisions");
    switch (robotMapType) {
        case MapRepresentation::DeterministicVoxelMap: {
            return getRobotDeterministicVoxelMapCollisions(robotMapName,
                environmentMapName,
                environmentMapType);
            break;
        }
        case MapRepresentation::DeterministicOctree: {
            return getRobotDeterministicOctreeCollisions(robotMapName,
                environmentMapName,
                environmentMapType);
            break;
        }
        case MapRepresentation::ProbabilisticVoxelMap: {
            return getRobotProbabilisticVoxelMapCollisions(robotMapName,
                environmentMapName,
                environmentMapType);
            break;
        }
        default: {
            logger_->error("The map representation is not implemented");
            return boost::none;
            break;
        }
    }
}

boost::optional<std::size_t> GPUVoxelsCollisionDetector::getRobotDeterministicVoxelMapCollisions(
    const std::string &robotMapName,
    const std::string &environmentMapName,
    const MapRepresentation &environmentMapType) {
    logger_->debug("getRobotDeterministicVoxelMapCollisions");
    voxelmap::BitVectorVoxelMap* robotMapVoxelMap =
        gpuVoxels_->getMap("RobotMap")->as<voxelmap::BitVectorVoxelMap>();
    switch (environmentMapType) {
        case MapRepresentation::DeterministicVoxelMap: {
            voxelmap::BitVectorVoxelMap* environmentMapVoxelMap =
                gpuVoxels_->getMap("EnvironmentMap")->as<voxelmap::BitVectorVoxelMap>();
            return robotMapVoxelMap->collideWith(environmentMapVoxelMap);
            break;
        }
        case MapRepresentation::DeterministicOctree: {
            NTree::GvlNTreeDet* environmentMapVoxelMap =
                gpuVoxels_->getMap("EnvironmentMap")->as<NTree::GvlNTreeDet>();
            return environmentMapVoxelMap->collideWith(robotMapVoxelMap);
            break;
        }
        case MapRepresentation::ProbabilisticVoxelMap: {
            voxelmap::ProbVoxelMap* environmentMapVoxelMap =
                gpuVoxels_->getMap("EnvironmentMap")->as<voxelmap::ProbVoxelMap>();
            return robotMapVoxelMap->collideWith(environmentMapVoxelMap,
                OCCUPANCY_THRESHOLD_FOR_COLLISIONS_IN_PROBABILISTIC_MAPS);
            break;
        }
        default: {
            logger_->error("The collision between this two maps is not implemented");
            return boost::none;
            break;
        }
    }
}

boost::optional<std::size_t> GPUVoxelsCollisionDetector::getRobotDeterministicOctreeCollisions(
    const std::string &robotMapName,
    const std::string &environmentMapName,
    const MapRepresentation &environmentMapType) {
    logger_->debug("getRobotDeterministicOctreeCollisions");
    NTree::GvlNTreeDet* robotMapVoxelMap =
        gpuVoxels_->getMap("RobotMap")->as<NTree::GvlNTreeDet>();
    switch (environmentMapType) {
        case MapRepresentation::DeterministicVoxelMap: {
            voxelmap::BitVectorVoxelMap* environmentMapVoxelMap =
                gpuVoxels_->getMap("EnvironmentMap")->as<voxelmap::BitVectorVoxelMap>();
            return robotMapVoxelMap->collideWith(environmentMapVoxelMap);
            break;
        }
        case MapRepresentation::DeterministicOctree: {
            NTree::GvlNTreeDet* environmentMapVoxelMap =
                gpuVoxels_->getMap("EnvironmentMap")->as<NTree::GvlNTreeDet>();
            return robotMapVoxelMap->collideWith(environmentMapVoxelMap);
            break;
        }
        case MapRepresentation::ProbabilisticVoxelMap: {
            voxelmap::ProbVoxelMap* environmentMapVoxelMap =
                gpuVoxels_->getMap("EnvironmentMap")->as<voxelmap::ProbVoxelMap>();
            return robotMapVoxelMap->collideWith(environmentMapVoxelMap,
                OCCUPANCY_THRESHOLD_FOR_COLLISIONS_IN_PROBABILISTIC_MAPS);
            break;
        }
        default: {
            logger_->error("The collision between this two maps is not implemented");
            return boost::none;
            break;
        }
    }
}

boost::optional<std::size_t> GPUVoxelsCollisionDetector::getRobotProbabilisticVoxelMapCollisions(
    const std::string &robotMapName,
    const std::string &environmentMapName,
    const MapRepresentation &environmentMapType) {
    logger_->debug("getRobotDeterministicOctreeCollisions");
    voxelmap::ProbVoxelMap* robotMapVoxelMap =
        gpuVoxels_->getMap("RobotMap")->as<voxelmap::ProbVoxelMap>();
    switch (environmentMapType) {
        case MapRepresentation::DeterministicVoxelMap: {
            voxelmap::BitVectorVoxelMap* environmentMapVoxelMap =
                gpuVoxels_->getMap("EnvironmentMap")->as<voxelmap::BitVectorVoxelMap>();
            return robotMapVoxelMap->collideWith(environmentMapVoxelMap,
                OCCUPANCY_THRESHOLD_FOR_COLLISIONS_IN_PROBABILISTIC_MAPS);
            break;
        }
        case MapRepresentation::DeterministicOctree: {
            NTree::GvlNTreeDet* environmentMapVoxelMap =
                gpuVoxels_->getMap("EnvironmentMap")->as<NTree::GvlNTreeDet>();
            return environmentMapVoxelMap->collideWith(robotMapVoxelMap,
                OCCUPANCY_THRESHOLD_FOR_COLLISIONS_IN_PROBABILISTIC_MAPS);
            break;
        }
        case MapRepresentation::ProbabilisticVoxelMap: {
            voxelmap::ProbVoxelMap* environmentMapVoxelMap =
                gpuVoxels_->getMap("EnvironmentMap")->as<voxelmap::ProbVoxelMap>();
            return robotMapVoxelMap->collideWith(environmentMapVoxelMap,
                OCCUPANCY_THRESHOLD_FOR_COLLISIONS_IN_PROBABILISTIC_MAPS);
            break;
        }
        default: {
            logger_->error("The collision between this two maps is not implemented");
            return boost::none;
            break;
        }
    }
}

boost::optional<float> GPUVoxelsCollisionDetector::getDistance(float initialValue,
    float finalValue,
    SpaceType type) const {
    logger_->debug("getDistance");
    float result = std::abs(initialValue - finalValue);
    switch (type) {
        case SpaceType::REALVECTOR_STATE_SPACE: {
            return result;
            break;
        }
        case SpaceType::SO2_STATE_SPACE: {
            if (std::abs(initialValue) > pi_ || std::abs(finalValue) > pi_) {
                logger_->error("The states are not within the SO(2) bounds");
                return boost::none;
                break;
            }
            if (result > pi_) {
                result = twopi_ - result;
            }
            return result;
            break;
        }
        case SpaceType::UNKNOWN_STATE_SPACE: {
            logger_->error("The state type is unknown");
            return boost::none;
            break;
        }
    }
    logger_->error("The state type is not implemented");
    return boost::none;
}

boost::optional<unsigned int> GPUVoxelsCollisionDetector::getNumberOfSegments(
    const std::vector<float> &initialState,
    const std::vector<float> &finalState,
    float resolution) const {
    logger_->debug("getNumberOfSegments");
    float result = 0;
    float currentNumber = 0;
    for (std::size_t dimID = 0; dimID < jointsNumber_; dimID++) {
        auto distance = getDistance(initialState[dimID],
            finalState[dimID],
            stateTypes_[dimID]);
        if (!distance) {
            logger_->error("Failed to calculate the distance");
            return boost::none;
        }
        currentNumber = distance.get()/resolution;
        if (result < currentNumber) {
            result = currentNumber;
        }
    }
    return static_cast<unsigned int>(ceil(result));
}

std::vector<float> GPUVoxelsCollisionDetector::interpolateStates(
    const std::vector<float> &initialState,
    const std::vector<float> &finalState,
    float time) const {
    logger_->debug("interpolateStates");
    std::vector<float> result(jointsNumber_);
    for (std::size_t dimID = 0; dimID < jointsNumber_; dimID++) {
        switch (stateTypes_[dimID]) {
            case SpaceType::REALVECTOR_STATE_SPACE: {
                result[dimID] = interpolateRealSpaceValues(initialState[dimID],
                    finalState[dimID],
                    time);
                break;
            }
            case SpaceType::SO2_STATE_SPACE: {
                result[dimID] = interpolateSO2SpaceValues(initialState[dimID],
                    finalState[dimID],
                    time);
                break;
            }
            case SpaceType::UNKNOWN_STATE_SPACE: {
                logger_->error("The state type is unknown");
                return std::vector<float>();
                break;
            }
        }
    }
    return result;
}

float GPUVoxelsCollisionDetector::interpolateRealSpaceValues(float initialValue,
    float finalValue,
    float time) const {
    logger_->debug("interpolateRealSpaceValues");
    float difference = finalValue-initialValue;
    return initialValue + difference*time;
}

float GPUVoxelsCollisionDetector::interpolateSO2SpaceValues(float initialValue,
    float finalValue,
    float time) const {
    logger_->debug("interpolateSO2SpaceValues");
    float result;
    float difference = finalValue-initialValue;

    if (std::abs(difference) <= pi_)
        return initialValue + difference*time;

    if (difference > 0.0)
        difference = twopi_ - difference;
    else
        difference = -twopi_ - difference;

    result = initialValue - difference*time;
    if (result > pi_)
        result -= twopi_;
    else if (result < -pi_)
        result += twopi_;

    return result;
}


}  // namespace crf::navigation::collisiondetector
