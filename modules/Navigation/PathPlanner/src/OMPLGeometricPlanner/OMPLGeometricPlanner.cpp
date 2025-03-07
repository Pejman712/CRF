/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playan Garai CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include <vector>
#include <memory>
#include <limits>

#include <nlohmann/json.hpp>

#include "PathPlanner/OMPLGeometricPlanner/OMPLGeometricPlanner.hpp"

namespace crf::navigation::pathplanner {

OMPLGeometricPlanner::OMPLGeometricPlanner(
    const nlohmann::json& configuration,
    const PathPlannerMethod& plannerMethod,
    const OptimizerMethod& optimizerMethod,
    std::shared_ptr<IStateValidator> stateValidator,
    std::shared_ptr<IMotionValidator> motionValidator) :
    logger_("OMPLGeometricPlanner") {
    logger_->debug("CTor");

    try {
        if (configuration.contains("ValidityCheckingResolution")) {
            validityCheckingResolution_ =
                configuration["ValidityCheckingResolution"].get<double>();
        } else {
            validityCheckingResolution_ = 0.01;  // 1% default of OMPL
            logger_->info("Selecting a resolution of 1% for state validity checking by default");
        }
        maximumPlanningTimeSeconds_ = configuration["MaximumPlanningTimeSeconds"].get<double>();
        maximumSimplificationTimeSeconds_ =
            configuration["MaximumSimplificationTimeSeconds"].get<double>();
        simplifyPath_ = configuration["SimplifyPath"].get<bool>();
    } catch (const std::exception& e) {
        throw std::invalid_argument(
            "OMPLGeometricPlanner - CTor - Configuration JSON incorrect: " +
            std::string(e.what()));
    }

    stateSpace_ = std::make_shared<OMPLStateSpaceConfiguration>(
        configuration["StateConfiguration"]);
    spaceInformation_ = std::make_shared<ompl::base::SpaceInformation>(stateSpace_);
    if (stateValidator_ != nullptr) {
        stateValidator_ = std::make_shared<OMPLStateValidatorBridge>(
            stateValidator, stateSpace_);
    }
    if (motionValidator_ != nullptr) {
        motionValidator_ = std::make_shared<OMPLMotionValidatorBridge>(
            motionValidator, stateSpace_);
    }
    spaceInformation_->setStateValidityChecker(stateValidator_);
    spaceInformation_->setStateValidityCheckingResolution(validityCheckingResolution_);
    spaceInformation_->setMotionValidator(motionValidator_);
    spaceInformation_->setup();
    problemDefinition_ = std::make_shared<ompl::base::ProblemDefinition>(spaceInformation_);
    problemDefinition_->setOptimizationObjective(createOptimizer(optimizerMethod));
    planner_ = createPlanner(plannerMethod);
}

OMPLGeometricPlanner::~OMPLGeometricPlanner() {
    logger_->debug("DTor");
}

crf::expected<std::vector<std::vector<double>>> OMPLGeometricPlanner::computePath(
    const std::vector<double>& start, const std::vector<double>& goal) {
    logger_->debug("computePath");

    // Parse start to state
    ompl::base::ScopedState<ompl::base::CompoundStateSpace> startState(stateSpace_);
    for (uint64_t i = 0; i < start.size(); i++) {
        startState[i] = start[i];
    }

    // Parse goal to state
    ompl::base::ScopedState<ompl::base::CompoundStateSpace> goalState(stateSpace_);
    for (uint64_t i = 0; i < start.size(); i++) {
        goalState[i] = goal[i];
    }

    problemDefinition_->setStartAndGoalStates(startState, goalState);
    planner_->setProblemDefinition(problemDefinition_);
    ompl::base::PlannerStatus status = planner_->solve(maximumPlanningTimeSeconds_);

    if (!status) {
        if (status == ompl::base::PlannerStatus::UNKNOWN) {
            logger_->error("Uninitialized status");
            return crf::Code::NotInitialized;
        }
        if (status == ompl::base::PlannerStatus::INVALID_START) {
            logger_->error("Invalid start state or no start state specified");
            return crf::Code::BadRequest;
        }
        if (status == ompl::base::PlannerStatus::INVALID_GOAL) {
            logger_->error("Invalid goal state");
            return crf::Code::BadRequest;
        }
        if (status == ompl::base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE) {
            logger_->error("The goal is of a type that a planner does not recognize");
            return crf::Code::BadRequest;
        }
        if (status == ompl::base::PlannerStatus::TIMEOUT) {
            logger_->error("The planner failed to find a solution in the desired time");
            return crf::Code::ThirdPartyQueryFailed;
        }
        if (status == ompl::base::PlannerStatus::CRASH) {
            logger_->info("The planner crashed");
            return crf::Code::ThirdPartyQueryFailed;
        }
        if (status == ompl::base::PlannerStatus::ABORT) {
            logger_->info("The planner did not find a solution for some other reason");
            return crf::Code::ThirdPartyQueryFailed;
        }
        logger_->error("Unknown error");
        return crf::Code::ThirdPartyQueryFailed;
    }
    if (status == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION) {
        logger_->warn("The planner found an approximate solution");
    }
    if (status == ompl::base::PlannerStatus::EXACT_SOLUTION) {
        logger_->info("The planner found an exact solution");
    }

    std::shared_ptr<ompl::base::Path> pathSolution = problemDefinition_->getSolutionPath();

    ompl::geometric::PathSimplifier simplifier(spaceInformation_);

    if (simplifyPath_) {
        simplifier.simplify(
            *pathSolution->as<ompl::geometric::PathGeometric>(),
            maximumSimplificationTimeSeconds_,
            false);  // At least once is set to false, don't optimize if not needed
    }

    ompl::geometric::PathGeometric geometricPath =
        *pathSolution->as<ompl::geometric::PathGeometric>();
    geometricPath.interpolate();

    std::vector<std::vector<double>> path;
    std::vector<double> reals;
    for (ompl::base::State* state : geometricPath.getStates()) {
        stateSpace_->copyToReals(reals, state);
        path.push_back(reals);
    }

    return path;
}

// Private

std::shared_ptr<ompl::base::Planner> OMPLGeometricPlanner::createPlanner(
    const PathPlannerMethod& method) {
    switch (method) {
        case PathPlannerMethod::PRM:
            return std::make_shared<ompl::geometric::PRM>(spaceInformation_);

        case PathPlannerMethod::LazyPRM:
            return std::make_shared<ompl::geometric::LazyPRM>(spaceInformation_);

        case PathPlannerMethod::PRMStar:
            return std::make_shared<ompl::geometric::PRMstar>(spaceInformation_);

        case PathPlannerMethod::LazyPRMStar:
            return std::make_shared<ompl::geometric::LazyPRMstar>(spaceInformation_);

        case PathPlannerMethod::SPARS:
            return std::make_shared<ompl::geometric::SPARS>(spaceInformation_);

        case PathPlannerMethod::SPARS2:
            return std::make_shared<ompl::geometric::SPARStwo>(spaceInformation_);

        case PathPlannerMethod::RRT:
            return std::make_shared<ompl::geometric::RRT>(spaceInformation_);

        case PathPlannerMethod::RRTConnect:
            return std::make_shared<ompl::geometric::RRTConnect>(spaceInformation_);

        case PathPlannerMethod::RRTStar:
            return std::make_shared<ompl::geometric::RRTstar>(spaceInformation_);

        case PathPlannerMethod::RRTSharp:
            return std::make_shared<ompl::geometric::RRTsharp>(spaceInformation_);

        case PathPlannerMethod::RRTXStatic:
            return std::make_shared<ompl::geometric::RRTXstatic>(spaceInformation_);

        case PathPlannerMethod::InformedRRTStar:
            return std::make_shared<ompl::geometric::InformedRRTstar>(spaceInformation_);

        case PathPlannerMethod::BITStar:
            return std::make_shared<ompl::geometric::BITstar>(spaceInformation_);

        case PathPlannerMethod::ABITStar:
            return std::make_shared<ompl::geometric::ABITstar>(spaceInformation_);

        case PathPlannerMethod::AITStar:
            return std::make_shared<ompl::geometric::AITstar>(spaceInformation_);

        case PathPlannerMethod::LBTRRT:
            return std::make_shared<ompl::geometric::LBTRRT>(spaceInformation_);

        case PathPlannerMethod::SST:
            return std::make_shared<ompl::geometric::SST>(spaceInformation_);

        case PathPlannerMethod::TRRT:
            return std::make_shared<ompl::geometric::TRRT>(spaceInformation_);

        case PathPlannerMethod::VectorFieldRRT:
            // Needs the vector field
            // return std::make_shared<ompl::geometric::VFRRT>(spaceInformation_);
            throw std::logic_error("Vector field RRT is not implemented");

        case PathPlannerMethod::ParallelRRT:
            return std::make_shared<ompl::geometric::pRRT>(spaceInformation_);

        case PathPlannerMethod::LazyRRT:
            return std::make_shared<ompl::geometric::LazyRRT>(spaceInformation_);

        case PathPlannerMethod::TaskSpaceRRT:
            // Needs the Task Space configuration
            // return std::make_shared<ompl::geometric::TSRRT>(spaceInformation_);
            throw std::logic_error("Task Space RRT is not implemented");

        case PathPlannerMethod::STRRTStar:
            // Could not include it
            // return std::make_shared<ompl::geometric::STRRTstar>(spaceInformation_);
            throw std::logic_error("ST-RRTStar is not available");

        case PathPlannerMethod::EST:
            return std::make_shared<ompl::geometric::EST>(spaceInformation_);

        case PathPlannerMethod::SBL:
            return std::make_shared<ompl::geometric::SBL>(spaceInformation_);

        case PathPlannerMethod::ParallelSBL:
            return std::make_shared<ompl::geometric::pSBL>(spaceInformation_);

        case PathPlannerMethod::KPIECE:
            return std::make_shared<ompl::geometric::KPIECE1>(spaceInformation_);

        case PathPlannerMethod::BKPIECE:
            return std::make_shared<ompl::geometric::BKPIECE1>(spaceInformation_);

        case PathPlannerMethod::LBKPIECE:
            return std::make_shared<ompl::geometric::LBKPIECE1>(spaceInformation_);

        case PathPlannerMethod::STRIDE:
            return std::make_shared<ompl::geometric::STRIDE>(spaceInformation_);

        case PathPlannerMethod::PDST:
            return std::make_shared<ompl::geometric::PDST>(spaceInformation_);

        case PathPlannerMethod::FMT:
            return std::make_shared<ompl::geometric::FMT>(spaceInformation_);

        case PathPlannerMethod::BFMT:
            return std::make_shared<ompl::geometric::BFMT>(spaceInformation_);

        case PathPlannerMethod::CForest:
            return std::make_shared<ompl::geometric::CForest>(spaceInformation_);

        default:
            throw std::logic_error("Selected path planner method is not available");
    }
}

ompl::base::OptimizationObjectivePtr OMPLGeometricPlanner::createOptimizer(
    const OptimizerMethod& method) {
    switch (method) {
        case OptimizerMethod::ConstraintObjective:
            throw std::logic_error("The optimization Constratint Objective is not implemented");

        case OptimizerMethod::ControlDuration:
            // This method should be available but I can't include it (jplayang)
            // return std::make_shared<ompl::base::ControlDurationObjective>(spaceInformation_);
            throw std::logic_error("The optimization Control Duration is not implemented");

        case OptimizerMethod::MechanicalWork:
            return std::make_shared<ompl::base::MechanicalWorkOptimizationObjective>(
                spaceInformation_);

        case OptimizerMethod::MinimaxObjective:
            return std::make_shared<ompl::base::MinimaxObjective>(spaceInformation_);

        case OptimizerMethod::MaximizeMinClearance:
            return std::make_shared<ompl::base::MaximizeMinClearanceObjective>(spaceInformation_);

        case OptimizerMethod::MinimizeArrivalTime:
            throw std::logic_error("The optimization Minimize Arrival time is not implemented");

        case OptimizerMethod::MultiOptimization:
            throw std::logic_error("The optimization Multi Optimization is not implemented");

        case OptimizerMethod::PathLength:
            return std::make_shared<ompl::base::PathLengthOptimizationObjective>(
                spaceInformation_);

        case OptimizerMethod::StateCostIntegral:
            return std::make_shared<ompl::base::StateCostIntegralObjective>(spaceInformation_);

        case OptimizerMethod::VFUpstreamCriterion:
            throw std::logic_error("The optimization VFUpstreamCriterion is not implemented");

        default:
            throw std::logic_error("Selected optimization method is not available");
    }
}

}  // namespace crf::navigation::pathplanner
