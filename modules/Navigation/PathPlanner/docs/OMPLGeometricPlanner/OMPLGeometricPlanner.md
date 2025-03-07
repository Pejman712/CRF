@ingroup group_ompl_geometric_planner

This is an implemnetation of a Path Planner that follows the IPathPlanner interface and uses the library [OMPL (Open Motion Planning Library)](https://ompl.kavrakilab.org/index.html).

OMPL is an open-source C++ library designed to facilitate motion planning in a variety of robotic systems and autonomous applications. Developed by the Computational Robotics Lab at the University of California, OMPL provides a comprehensive set of tools and algorithms for solving complex motion planning problems.

### State Space

OMPL provides built-in state spaces for common scenarios, including Euclidean spaces, SE(2) (2D rigid body transformations), SE(3) (3D rigid body transformations), SO(2) (2D rotations), and SO(3) (3D rotations). Users can define custom state spaces to represent specific configurations or scenarios unique to their robotic systems. This flexibility is crucial for handling non-standard configurations.

State spaces can have bounds defined for each dimension, constraining the valid values a state can take. Bounds are essential for ensuring that the generated paths are feasible within the specified limits.

In order to facilitate the construction of this state spaces specific to each robot, the class StateSpaceConfiguration transforms a JSON configuration file into an state space.

### State Validator

If not provided through the construtor the path won't be checked.

### Motion Validator

If not provided, the library defaults to a Discrete Motion Validator that divides the two states into smaller parts and analizes them with the state validator.

### Usage

\code{.cpp}
nlohmann::json configurationFileJSON;  // json file
std::shared_ptr<IOMPLStateValidator> stateValidator = nullptr;
std::shared_ptr<IOMPLMotionValidator> motionValidator = nullptr;

std::unique_ptr<OMPLGeometricPlanner> planner =
    std::make_unique<OMPLGeometricPlanner>(
        configurationFileJSON,
        PathPlannerMethod::RRTStar,
        OptimizerMethod::PathLength,
        stateValidator,
        motionValidator);

std::vector<double> start = {0, 0, 0};
std::vector<double> goal = {1e3, 1e3, 2};
auto path = planner.computePath(start, goal);

if (!path) {
    // Manage error
}
// Path succesfuly created

\endcode

#### Parameters

The configuration file must follow the next structure:

\code{.json}
{
    "ValidityCheckingResolution" : 0.03,  // 3%
    "MaximumPlanningTimeSeconds" : 1,
    "MaximumSimplificationTimeSeconds" : 1,
    "SimplifyPath" : true,
    "StateConfiguration" : [
        {
            "Type" : "Real",
            "Dimension" : 1,
            "Weight" : 1,
            "Bounds" : {
                "Max" : 1e4,
                "Min" : -1e4
            }
        },
        {
            "Type" : "Real",
            "Dimension" : 1,
            "Weight" : 1,
            "Bounds" : {
                "Max" : 1e4,
                "Min" : -1e4
            }
        },
        {
            "Type" : "SO2",
            "Weight" : 1
        }
    ]
}
\endcode

Where:

- **ValidityCheckingResolution**: states the resolution for the Discrete Motion validator (the one used by default). It's used to specify the maximum distance between states to be checked for validity along a path segment. This distance is specified as a percentage of a space's maximum extent. If this call is not made, the resolution is assumed to be 1%. This value may be too low, in which case planning will be slower, or it may be too high, in which case it is possible to have collisions in solution plans.
- **MaximumPlanningTimeSeconds**: the maximum time alloewed to the planner
- **SimplifyPath**: if you want to simplify the path.
- **MaximumSimplificationTimeSeconds**: the maximum time allowed to the simplifier to try to simplify the path (only if simplify path is true).
- **StateConfiguration**: The state space configuration of your planning space. This can vary for each robot. To know how to define it follow StateSpaceConfiguration

The next parameter is the PathPlannerMethod. With it you can select the method you want, bear in mind that not all methods work with all state spaces. Carefully select the method you want to use.

As for OptimizerMethod the most common one is OptimizerMethod::PathLength, but it's not the ony one available.

Feel free to check the list of available optimizers and planners.

The state validator and motion validator are optional fields. If not specified they won't be used (most often resulting in straight lines for optimal planners).
