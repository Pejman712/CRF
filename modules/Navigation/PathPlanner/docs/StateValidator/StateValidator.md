@ingroup group_state_validator

The `IStateValidator` class defines an interface for state validation in the context of path planning. It provides a method to check whether a given state is valid or in collision with the environment.

#### Usage Example

\code{.cpp}
// Example Usage:
std::unique_ptr<IStateValidator> validator = std::make_unique<MyStateValidator>();

std::vector<double> currentState = {1.0, 2.0, 3.0};

if (validator->isStateValid(currentState)) {
    // The current state is valid and collision-free
    // Proceed with further actions...
} else {
    // The current state is invalid or in collision
    // Handle the error or adjust the state...
}
/endcode
