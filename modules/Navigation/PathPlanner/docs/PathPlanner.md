@ingroup group_path_planner

Path planning, also known as motion planning or trajectory planning, is a fundamental concept in the field of robotics and artificial intelligence. It involves finding a feasible path for a robot or agent to navigate from an initial state to a goal state in an environment while avoiding obstacles and satisfying constraints.

This module aims to find a common interface that all possible path planners can use.

### IPathPlanner Class

The `IPathPlanner` class defines an interface for path planning algorithms. It is designed to calculate a path from a start state to a goal state in an N-dimensional space. Every time the `computePath` method is called, a new path is calculated based on the given start and goal positions.

#### ComputePath

Calculates a path from the specified start state to the goal state.

- **Parameters:**
  - `start`: Starting position in N dimensions.
  - `goal`: Goal position in N dimensions.

- **Returns:**
  - A `crf::expected` containing either the calculated path in N dimensions (`std::vector<std::vector<double>>`) or an error code indicating why the path calculation failed.

### State Space

Path planners use the concept of state space to refer to the configuration space of the robot. This space is fundamental to the generation of a path.

State spaces can have bounds defined for each dimension, constraining the valid values a state can take. Bounds are essential for ensuring that the generated paths are feasible within the specified limits.

### State Validator

The state validator takes care of understanding if a path is valid or not, for example checked for collisions.

### Motion Validator

A motion validator is a crucial component used in path planning algorithms to check the validity of motions or transitions between states in the configuration space. The primary role of a motion validator is to determine whether a proposed motion segment is collision-free, ensuring that the robot or system can move smoothly from one state to another without encountering obstacles.

#### Usage Example

\code{.cpp}
// Example Usage:
std::unique_ptr<IPathPlanner> planner = std::make_unique<MyPathPlanner>();

std::vector<double> start = {0.0, 0.0, 0.0};
std::vector<double> goal = {5.0, 5.0, 5.0};

auto result = planner->computePath(start, goal);

if (result) {
    // Successfully calculated path
    std::vector<std::vector<double>> path = result.value();
    // Process the path...
} else {
    // Path planning failed, handle the error
    std::cerr << "Path planning error: " << result.get_response() << std::endl;
}
/endcode
