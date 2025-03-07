@ingroup group_joint_types

Class [VectorXd](@ref crf::utility::types::VectorXd) 
is intended class to use in the CRF, to inherit from, when implementing strongly typed classes
that acts as vectors of variable length.

Classes:
    - [JointPositions](@ref crf::utility::types::JointPositions) 
    - [JointVelocities](@ref crf::utility::types::JointVelocities)
    - [JointAccelerations](@ref crf::utility::types::JointAccelerations)
    - [JointForceTorques](@ref crf::utility::types::JointForceTorques)
are inhriting from this class.

Arithmetical operators are not implemented as a member functions. Instead all
arithmetical operations can be performed by first extractng the
[Eigen::VectorXd]() object with the [raw](@ref crf::utility::types::VectorXd::raw())
operator.

Values stored can be an ordinary double or one of three special values:
    - std::numeric_limits<double>::infinity()
    - -std::numeric_limits<double>::infinity()
    - std::numeric_limits<double>::quier_NaN()
