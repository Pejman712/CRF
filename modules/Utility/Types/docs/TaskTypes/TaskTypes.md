@ingroup group_task_types

### TaskPose

#### Orientation

In the TaskPose class, orientation is represented as an Orientation class,
which is a different name, used in the context of TaskPose,
for the [Rotation](@ref crf::math::rotation::Rotation) class.

All the discussion about handling different orientation representations, can be found
in the documentation of [Rotation module](@ref group_rotation).

#### Representations

Supported representations consist of:

- a vector of three numbers for a position and either of the representations
supported by [Rotation](@ref crf::math::rotation::Rotation) class
- a homogeneous transformation matrix

##### Homogeneous transformation matrix

In the constructor and the setter, provided matrix is checked whether it is a valid
homogeneous transformation matrix up to the specified parameter 'accuracy'.

### Linear TaskTypes

Class [Vector6d](@ref crf::utility::types::Vector6d)
is intended class to use in the CRF, to inherit from, when implementing strongly typed classes
that acts as vectors of the length six.

Classes:
    - [TaskVelocity](@ref crf::utility::types::TaskVelocity)
    - [TaskAcceleration](@ref crf::utility::types::TaskAcceleration)
    - [TaskForceTorque](@ref crf::utility::types::TaskForceTorque)
are inhriting from this class.

Arithmetical operators are not implemented as a member functions. Instead all
arithmetical operations can be performed by first extractng the
[Eigen::Vector<double, 6>]() object with the [raw](@ref crf::utility::types::Vector6d::raw())
operator.

Classes:
    - [TaskVelocity](@ref crf::utilty::types::TaskVelocity)
    - [TaskAcceleration](@ref crf::utilty::types::TaskAcceleration)
    - [TaskForceTorques](@ref crf::utilty::types::TaskForceTorques)
all inherit from this class.

**Important convention** -- in the whole CRF, the usage of these types follows the convention
that three linear dimensions come first, followed up by three angular dimensions.

There are no specific types for linear and angular parts of these separately.
Usually [Eigen::Vector3d]() is used then.
Linear part can be extracted as
```cpp
vector.head(3)
```
and angular part as
```cpp
vector.tail(3)
```

Values stored can be an ordinary double or one of three special values:
 - std::numeric_limits<double>::infinity()
 - -std::numeric_limits<double>::infinity()
 - std::numeric_limits<double>::quier_NaN()

### TaskSpace

This is intended class to use in the CRF to describe which tangent dimensions in the
task space are used.

The coordinates in this class are enumerated by
[TaskSpaceTangentDimensions](@ref crf::utility::types::TaskSpaceTangentDimensions)
and corrspond to dimensions in velocity, acceleration and force&torque.
It is correspondint to directions at any given pose point, but they do not
represent any global dimension in the task space. As such
it can't be used in the same way as a dimensions of TaskPose.

#### TaskSpaceTangentDimension

THe enum class
[TaskSpaceTangentDimensions](@ref crf::utility::types::TaskSpaceTangentDimensions)
consists of six values:
    - 'Vx' -- linear direction along x axis
    - 'Vy' -- linear direction along y axis
    - 'Vz' -- linear direction along z axis
    - 'Wx' -- angular direction around x axis
    - 'Wy' -- angular direction around y axis
    - 'Wz' -- angular direction around z axis

#### Getters

There are various representations available via getters.

Representations with matrices return matrices which transforms a vector multiplied by them, to the vector of certain properties.
On examples:

For the task space defined as:
    - 'Vx': true
    - 'Vy': true
    - 'Vz': false
    - 'Wx': false
    - 'Wy': false
    - 'Wz': true

Possible matrices are:
    - NoRowsMatrix:
    \f[
        \begin{pmatrix}
        1 & 0 & 0 & 0 & 0 & 0 \\
        0 & 1 & 0 & 0 & 0 & 0 \\
        0 & 0 & 0 & 0 & 0 & 1
        \end{pmatrix}
    \f]
    - ZeroRowsMatrix:
    \f[
        \begin{pmatrix}
        1 & 0 & 0 & 0 & 0 & 0 \\
        0 & 1 & 0 & 0 & 0 & 0 \\
        0 & 0 & 0 & 0 & 0 & 0 \\
        0 & 0 & 0 & 0 & 0 & 0 \\
        0 & 0 & 0 & 0 & 0 & 0 \\
        0 & 0 & 0 & 0 & 0 & 1
        \end{pmatrix}
    \f]
    - NaNRowsMatrix:
    \f[
        \begin{pmatrix}
        1 & 0 & 0 & 0 & 0 & 0 \\
        0 & 1 & 0 & 0 & 0 & 0 \\
        0 & 0 & NaN & 0 & 0 & 0 \\
        0 & 0 & 0 & NaN & 0 & 0 \\
        0 & 0 & 0 & 0 & NaN & 0 \\
        0 & 0 & 0 & 0 & 0 & 1
        \end{pmatrix}.
    \f]
Where 'NaN' means here
```cpp
std::numeric_limits<double>::quier_NaN().
```

Then, for a vector \f$\begin{pmatrix}2.2\\ -3.8\\ 4.7\\ 5.8\\ 4.7\\ 3.9\end{pmatrix}\f$,
the result of multiplying this vector by these matrices are:
    - for NoRowsMatrix:
    \f[
        \begin{pmatrix}
        1 & 0 & 0 & 0 & 0 & 0 \\
        0 & 1 & 0 & 0 & 0 & 0 \\
        0 & 0 & 0 & 0 & 0 & 1
        \end{pmatrix}
        \begin{pmatrix}2.2\\ -3.8\\ 4.7\\ 5.8\\ 4.7\\ 3.9\end{pmatrix} =
        \begin{pmatrix}2.2\\ -3.8\\ 3.9\end{pmatrix}
    \f]
    -for ZeroRowsMatrix:
    \f[
        \begin{pmatrix}
        1 & 0 & 0 & 0 & 0 & 0 \\
        0 & 1 & 0 & 0 & 0 & 0 \\
        0 & 0 & 0 & 0 & 0 & 0 \\
        0 & 0 & 0 & 0 & 0 & 0 \\
        0 & 0 & 0 & 0 & 0 & 0 \\
        0 & 0 & 0 & 0 & 0 & 1
        \end{pmatrix}
        \begin{pmatrix}2.2\\ -3.8\\ 4.7\\ 5.8\\ 4.7\\ 3.9\end{pmatrix} =
        \begin{pmatrix}2.2\\ -3.8\\ 0.0 \\ 0.0 \\ 0.0\\ 3.9\end{pmatrix}
    \f]
    -for NaNRowsMatrix:
    \f[
        \begin{pmatrix}
        1 & 0 & 0 & 0 & 0 & 0 \\
        0 & 1 & 0 & 0 & 0 & 0 \\
        0 & 0 & NaN & 0 & 0 & 0 \\
        0 & 0 & 0 & NaN & 0 & 0 \\
        0 & 0 & 0 & 0 & NaN & 0 \\
        0 & 0 & 0 & 0 & 0 & 1
        \end{pmatrix}\begin{pmatrix}2.2\\ -3.8\\ 4.7\\ 5.8\\ 4.7\\ 3.9\end{pmatrix} =
        \begin{pmatrix}2.2\\ -3.8\\ NaN \\ NaN \\ NaN\\ 3.9\end{pmatrix}.
    \f]





