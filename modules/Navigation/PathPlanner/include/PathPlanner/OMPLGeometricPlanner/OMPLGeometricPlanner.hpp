/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playan Garai CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <vector>
#include <memory>

#include <nlohmann/json.hpp>

#include <ompl/base/StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/Planner.h>
#include <ompl/base/Path.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/PathGeometric.h>

// Planners
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/RRTXstatic.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/informedtrees/ABITstar.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/sst/SST.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/VFRRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/TSRRT.h>
// #include <ompl/geometric/planners/rrt/STRRTstar.h>  // Not available
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/sbl/pSBL.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/stride/STRIDE.h>
#include <ompl/geometric/planners/pdst/PDST.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/cforest/CForest.h>

// Optimizers
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/Constraint.h>
// #include <ompl/base/objectives/ControlDurationObjective.h>  // Not available
#include <ompl/base/objectives/MechanicalWorkOptimizationObjective.h>
#include <ompl/base/objectives/MinimaxObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/VFUpstreamCriterionOptimizationObjective.h>

#include "PathPlanner/IPathPlanner.hpp"
#include "PathPlanner/PathPlannerMethod.hpp"
#include "PathPlanner/OptimizerMethod.hpp"
#include "PathPlanner/OMPLGeometricPlanner/OMPLStateValidatorBridge.hpp"
#include "PathPlanner/OMPLGeometricPlanner/OMPLMotionValidatorBridge.hpp"
#include "PathPlanner/OMPLGeometricPlanner/OMPLStateSpaceConfiguration.hpp"

#include "EventLogger/EventLogger.hpp"

namespace crf::navigation::pathplanner {

/**
 * @ingroup group_ompl_geometric_planner
 * @brief IPathPlanner implemnetation using the OMPL library. This library
 * implements lots of path planning algortihms that we can freely use with
 * this class.
 *
 */
class OMPLGeometricPlanner: public IPathPlanner {
 public:
    /**
     * @brief Construct a new OMPLGeometricPlanner object.
     *
     * @param configuration Configuration file from which we generate the state space of the
     * robot.
     * @param plannerMethod Selection of the path planning algorithm.
     * @param optimizerMethod Selection of the optimizer method.
     * @param stateValidator Possible state validator to check the path. If not provided
     * the path is not checked
     * @param motionValidator Possible motion validator to check the motion between states. If
     * not provided the path get discretly cut and checked with the state validator.
     */
    OMPLGeometricPlanner(
        const nlohmann::json& configuration,
        const PathPlannerMethod& plannerMethod,
        const OptimizerMethod& optimizerMethod,
        std::shared_ptr<IStateValidator> stateValidator = nullptr,
        std::shared_ptr<IMotionValidator> motionValidator = nullptr);
    ~OMPLGeometricPlanner() override;

    crf::expected<std::vector<std::vector<double>>> computePath(
        const std::vector<double>& start, const std::vector<double>& goal) override;

 private:
    std::shared_ptr<ompl::base::Planner> createPlanner(const PathPlannerMethod& method);
    ompl::base::OptimizationObjectivePtr createOptimizer(const OptimizerMethod& method);

    std::shared_ptr<OMPLStateSpaceConfiguration> stateSpace_;
    std::shared_ptr<OMPLStateValidatorBridge> stateValidator_;
    std::shared_ptr<OMPLMotionValidatorBridge> motionValidator_;
    std::shared_ptr<ompl::base::SpaceInformation> spaceInformation_;
    std::shared_ptr<ompl::base::ProblemDefinition> problemDefinition_;
    std::shared_ptr<ompl::base::Planner> planner_;

    double validityCheckingResolution_;
    double maximumPlanningTimeSeconds_;
    double maximumSimplificationTimeSeconds_;
    bool simplifyPath_;

    crf::utility::logger::EventLogger logger_;
};

}  // namespace crf::navigation::pathplanner
