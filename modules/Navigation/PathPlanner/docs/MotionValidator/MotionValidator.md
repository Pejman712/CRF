@ingroup group_motion_validator

The `IMotionValidator` class defines an interface for motion validation in the context of path planning. It provides a method to check whether the trajectory segment between two states is valid, for example collision-free.

#### Usage Example

\code{.cpp}
// Example Usage:
std::unique_ptr<IMotionValidator> motionValidator = std::make_unique<MyMotionValidator>();

std::vector<double> startState = {1.0, 2.0, 3.0};
std::vector<double> goalState = {4.0, 5.0, 6.0};

if (motionValidator->isMotionValid(startState, goalState)) {
    // The motion segment between start and goal states is valid and collision-free
    // Proceed with further actions...
} else {
    // The motion segment is invalid or in collision
    // Handle the error or adjust the trajectory...
}
/endcode
