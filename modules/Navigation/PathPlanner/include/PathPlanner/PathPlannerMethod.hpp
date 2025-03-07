/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN EN/SMM/MRO 2019
 *         Jorge Playan Garai CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */
#pragma once

namespace crf::navigation::pathplanner {

/**
 * @ingroup group_path_planner
 * @enum PathPlannerMethod
 * @brief Enumeration representing various path planning algorithms.
 *
 * This enum defines different methods that can be used for path planning in a robotic system.
 * Each enumerator corresponds to a specific path planning algorithm.
 */
enum class PathPlannerMethod {
    /**
     * @brief No path planning algorithm defined.
     *
     * The NotDefined enumerator indicates that no specific path planning algorithm is selected.
     */
    NotDefined = 0,

    /**
     * @brief Probabilistic Roadmap (PRM) algorithm.
     *
     * PRM builds a roadmap of the configuration space, allowing efficient path planning in
     * complex environments. It's suitable for static environments with known obstacles.
     */
    PRM = 1,

    /**
     * @brief Lazy version of the Probabilistic Roadmap (PRM) algorithm.
     *
     * LazyPRM incrementally builds the roadmap, deferring the construction of edges until
     * needed for planning. It's suitable for dynamic environments with changing obstacles.
     */
    LazyPRM = 2,

    /**
     * @brief PRM* algorithm, an extension of the Probabilistic Roadmap algorithm.
     *
     * PRM* improves upon PRM by optimizing the roadmap for more efficient paths. PRM* is
     * useful when high-quality paths are required, and there is a need for optimizing the roadmap.
     */
    PRMStar = 3,

    /**
     * @brief Lazy version of the PRM* algorithm.
     *
     * LazyPRMStar is a lazy variant of PRM* that defers edge construction until required during
     * planning. It's suitable for dynamic environments where the roadmap needs to be optimized
     * on-the-fly.
     */
    LazyPRMStar = 4,

    /**
     * @brief Sparse Roadmap (SPARS) algorithm.
     *
     * SPARS builds a sparse roadmap, reducing the number of nodes while preserving connectivity
     * for efficient planning. It's beneficial in high-dimensional spaces where a sparse roadmap
     * can significantly reduce planning time.
     */
    SPARS = 5,

    /**
     * @brief Sparse Roadmap (SPARS) algorithm with enhanced performance.
     *
     * SPARS2 further optimizes the SPARS algorithm for improved efficiency in path planning.
     * It's suitable for scenarios where performance is a critical factor.
     */
    SPARS2 = 6,

    /**
     * @brief Rapidly-Exploring Random Tree (RRT) algorithm.
     *
     * RRT explores the configuration space with random samples, rapidly expanding towards
     * unexplored areas. It's effective in complex, high-dimensional spaces with rapidly
     * changing environments.
     */
    RRT = 7,

    /**
     * @brief RRTConnect algorithm for connecting two points with an RRT.
     *
     * RRTConnect focuses on efficiently connecting two points in the configuration space
     * using RRT exploration. It's suitable for scenarios where a connection between two points
     * needs to be found quickly.
     */
    RRTConnect = 8,

    /**
     * @brief RRT* algorithm, an extension of the Rapidly-Exploring Random Tree algorithm.
     *
     * RRT* improves the efficiency of RRT by optimizing the tree structure for better paths.
     * It's beneficial when high-quality paths and optimized exploration are priorities.
     */
    RRTStar = 9,

    /**
     * @brief RRT# algorithm, an extension of the Rapidly-Exploring Random Tree algorithm.
     *
     * RRT# improves the convergence rate of RRT. It's an asymptotically-optimal incremental
     * sampling-based motion planning algorithm.
     */
    RRTSharp = 10,

    /**
     * @brief RRTXstatic is an asymptotically-optimal incremental sampling-based motion
     * planning algorithm.
     *
     * It differs from the RRT* algorithm by maintaining a pseudo-optimal tree.
     */
    RRTXStatic = 11,

    /**
     * @brief Run RRT* with an informed search strategy that uses heuristics to only consider
     * subproblem that could provide a better solution.
     *
     * The search is limited to this subproblem by pruning the graph, generating samples
     * only in this subproblem
     */
    InformedRRTStar = 12,

    /**
     * @brief BIT* (Batch Informed Trees) is an anytime asymptotically optimal sampling-based
     * planning algorithm.
     *
     * It approaches problems by assuming that a simple solution exists and only goes onto
     * consider complex solutions when that proves incorrect. It accomplishes this by using
     * heuristics to search in order of decreasing potential solution quality.
     */
    BITStar = 13,

    /**
     * @brief ABIT* (Advanced Batch Informed Trees) is an almost-surely asymptotically optimal
     * path planner. It views the planning problem as the two subproblems of approximation and
     * search.
     *
     * This perspective allows it use advanced graph-search techniques, such as inflation and truncation.
     */
    ABITStar = 14,

    /**
     * @brief AIT* (Adaptively Informed Trees) is an almost-surely asymptotically optimal path planner.
     *
     * It aims to find an initial solution quickly and asymptotically converge to the globally optimal solution.
     */
    AITStar = 15,

    /**
     * @brief Lifelong Planning A* with RRT (LBTRRT) algorithm.
     *
     * LBTRRT combines A* search with RRT exploration for lifelong planning in dynamic
     * environments. It's useful for lifelong planning in dynamic environments where paths
     * need to be continuously updated.
     */
    LBTRRT = 16,

    /**
     * @brief Single-Query Sparse Tree (SST) algorithm.
     *
     * SST builds a sparse tree in the configuration space, focusing on single-query path
     * planning scenarios. It's suitable for scenarios where planning is primarily single-query
     * and a sparse tree is beneficial.
     */
    SST = 17,

    /**
     * @brief Transition-based RRT (TRRT) algorithm.
     *
     * TRRT extends RRT by incorporating transition tests to improve the exploration of the
     * configuration space. It's effective when the exploration needs to consider transitions
     * between different modes or states.
     */
    TRRT = 18,

    /**
     * @brief Vector Field RRT algorithm.
     *
     * VectorFieldRRT utilizes vector fields to guide the exploration of the configuration space
     * for efficient planning. It's suitable when the configuration space can be guided by a vector
     * field for efficient planning.
     */
    VectorFieldRRT = 19,

    /**
     * @brief Parallel RRT algorithm for multi-threaded planning.
     *
     * ParallelRRT employs parallelization to accelerate RRT exploration in multi-core environments.
     */
    ParallelRRT = 20,

    /**
     * @brief Lazy Rapidly-Exploring Random Tree (RRT) algorithm.
     *
     * LazyRRT incrementally constructs the tree, deferring edge expansion until necessary during
     * planning. It's beneficial in multi-core environments where parallelization can significantly
     * improve planning speed.
     */
    LazyRRT = 21,

    /**
     * @brief Task-Space RRT algorithm.
     *
     * TaskSpaceRRT plans in task space, allowing for more natural and intuitive specification of
     * robot motions. It's useful when the exploration needs to be deferred until specific paths
     * are required and when planning needs to be performed in a higher-level task space.
     */
    TaskSpaceRRT = 22,

    /**
     * @brief ST-RRT* is a bidirectional, time-optimal planner for planning in space-time.
     *
     * It operates similar to a bidirectional version of RRT*, but allows planning in unbounded time
     * spaces by gradual time-bound extensions and is highly optimized for planning in space-time by
     * employing Conditional Sampling and Simplified Rewiring.
     */
    STRRTStar = 23,

    /**
     * @brief Expansive Space Trees (EST) algorithm.
     *
     * EST builds expansive trees in the configuration space, facilitating efficient path planning.
     * It's beneficial when expansive trees are preferred for exploration in large configuration
     * spaces.
     */
    EST = 24,

    /**
     * @brief Sparse Bundle Learning (SBL) algorithm.
     *
     * SBL utilizes sparse bundles of paths to efficiently explore the configuration space for path
     * planning. It's effective when paths can be represented as bundles, reducing the exploration
     * complexity.
     */
    SBL = 25,

    /**
     * @brief Parallel Sparse Bundle Learning (SBL) algorithm.
     *
     * ParallelSBL extends SBL by leveraging parallelization for faster exploration in multi-core systems.
     */
    ParallelSBL = 26,

    /**
     * @brief Kinematic Planning by Interior-Exterior Cell Exploration (KPIECE) algorithm.
     *
     * KPIECE explores the configuration space using a divide-and-conquer strategy for efficient planning.
     */
    KPIECE = 27,

    /**
     * @brief Bi-directional KPIECE algorithm.
     *
     * BKPIECE extends KPIECE to plan bidirectionally, improving the exploration of the
     * configuration space.It's useful when planning needs to be performed bidirectionally for
     * improved efficiency.
     */
    BKPIECE = 28,

    /**
     * @brief Lazy Bi-directional KPIECE algorithm.
     *
     * LBKPIECE is a lazy variant of the Bi-directional KPIECE algorithm, deferring edge expansion
     * until necessary. It's suitable when lazy evaluation is desired in a bidirectional
     * exploration strategy.
     */
    LBKPIECE = 29,

    /**
     * @brief STRIDE algorithm for dynamic replanning.
     *
     * STRIDE focuses on dynamic replanning, adapting to changes in the environment for robust path planning.
     */
    STRIDE = 30,

    /**
     * @brief Probabilistic Roadmap Dual Space Trees (PDST) algorithm.
     *
     * PDST constructs dual space trees in the configuration space, combining PRM and RRT techniques.
     */
    PDST = 31,

    /**
     * @brief Fast Marching Trees (FMT) algorithm.
     *
     * FMT builds trees using a fast marching method, providing efficient paths in static environments.
     */
    FMT = 32,

    /**
     * @brief Bidirectional Fast Marching Trees (BFMT) algorithm.
     *
     * BFMT extends FMT to plan bidirectionally, improving exploration in both directions for better paths.
     */
    BFMT = 33,

    /**
     * @brief CForest (Coupled Forest of Random Engrafting Search Trees) is a parallelization framework
     * that is designed for single-query shortest path planning algorithms.
     *
     * It is not a planning algorithm per se.
     */
    CForest = 34
};

}  // namespace crf::navigation::pathplanner
