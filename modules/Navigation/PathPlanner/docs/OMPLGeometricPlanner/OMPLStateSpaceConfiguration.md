@ingroup group_ompl_state_space

The StateSpaceConfiguration class is designed to create compound state spaces for motion planning using OMPL (Open Motion Planning Library). It reads a configuration in JSON format and creates a compound state space based on the specified subspaces, each with its own type, dimension, bounds, and weight.

The JSON configuration should be an array of objects, where each object represents a subspace of the compound state space. Each subspace is defined by its type, dimension, optional bounds, and weight.
Subspace Types:

- "Real": A real vector state space.
- "SO2": A special orthogonal group in 2D (rotation in a plane).
- "SO3": A special orthogonal group in 3D (rotation in 3D space).
- "SE2": A 2D rigid body state space (translation and rotation in a plane).
- "SE3": A 3D rigid body state space (translation and rotation in 3D space).

Example JSON Configuration:

\code{.json}
[
    {
        "Type": "Real",
        "Dimension": 2,
        "Bounds": {
            "Min": -1.0,
            "Max": 1.0
        },
        "Weight": 1.0
    },
    {
        "Type": "SO2",
        "Weight": 0.5
    },
    {
        "Type": "SO3",
        "Weight": 0.5
    },
    {
        "Type": "SE2",
        "Bounds": {
            "Min": -2.0,
            "Max": 2.0
        },
        "Weight": 2.0
    }
    {
        "Type": "SE3",
        "Bounds": {
            "Min": -2.0,
            "Max": 2.0
        },
        "Weight": 2.0
    }
]
/endcode

The configuration for each subspace includes the following properties:

- "Type" (string): The type of the subspace (e.g., "Real", "SO2", etc.).
- "Dimension" (integer): The dimension of the subspace.
- "Bounds" (object, optional): Bounds for the subspace (applicable for real vector, SE2, and SE3 spaces).
- "Bounds.Min" (number): The minimum bound value.
- "Bounds.Max" (number): The maximum bound value.
- "Weight" (number): The weight assigned to the subspace in the compound state space.
