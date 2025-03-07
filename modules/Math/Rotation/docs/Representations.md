@ingroup group_rotation_representations

There are five representations of rotations supported:
    - [Quaternion](#Quaternion)
    - [Matrix](#Matrix)
    - [AngleAxis](#AngleAxis)
    - [CardanXYZ](#CardanXYZ)
    - [EulerZXZ](#EulerZXZ)

Overview and details of how different rotation representations work can be found for example at:
[Rotation formalism in three dimensions](https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions).

In crf the intended way for expressing these representations is through
[RotationRepresentation](@ref crf::math::rotation::RotationRepresentation) enum class.

\anchor additional_data

### Additional data in some representations

In case of AngleAxis, CardanXYZ and EulerZXZ, if we allow the coordinates to run through
whole \f$\mathbb{R}\f$, there are infinitely many presentations of a given rotation,
they differ from each other by multiplicity of \f$2\pi\f$ radians on each coordinate.
While describing the same rotation, it may be useful to track data of e.g.
rotating coordinate frame and plot it without jumps (discontinuities) on the plot.

There is also no straightforward meaningful way to translate this additional data
between AngleAxis, CardanXYZ and EulerZXZ representations.
For this reason, conversions from either AngleAxis, CardanXYZ or EulerZXZ to any other
representation (including to each other) is treated as a lossy process.

\anchor quaternion_representation

### Quaternion

Represents rotation as a unitary
[quaternion](https://en.wikipedia.org/wiki/Quaternion).
Details on how spacial rotation can be represented by quaternion can be found for exemple at:
[Quaternions and spacial rotation](https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation).

Throughut the CRF the convention for writing quaternions is that the real part is the first
coordinate when the representation is converted to any array type
and it is named "w". So, then the coordinates are ordered as follows:

- w
- x
- y
- z

\anchor matrix_representation

### Matrix

Represents rotation as a rotation matrix.
Details on how spacial rotation can be represented as rotation matrix can be found for exemple at:
[Rotation matrix in three dimensions](https://en.wikipedia.org/wiki/Rotation_matrix#In_three_dimensions).

\anchor angleAxis_representation

### AngleAxis

Represents rotation in terms of rotation axis and the angle around this axis.
More on how spacial rotations can be represented by angleAxis can be found for example at:
[Angle-Axis representation](https://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation).
Note, that in CRF we are adopting different convention than in the linked article,
calling this representation (after
[Eigen::AngleAxisd](https://eigen.tuxfamily.org/dox/classEigen_1_1AngleAxis.html))
"Angle-Axis", not "Axis-Angle".

Throughout the CRF the convention for writing AngleAxis representation is that the angle
comes first when the representation is converted to any array type. So then
coordinates goes as follows:

- angle
- x coordinate of the axis
- y coordinate of the axis
- z coordinate of the axis

Note: In the Eigen library, the convention for the zero rotation in the angleAxis representation is
that the angle is 0.0 and the axis is (1.0, 0.0, 0.0).

### Euler/Cardan angles

Represents rotation in terms of three angles.
More on how spacial rotations can be represented by Euler angles
(and information about naming conventions, which representation are "Euler angles" and which are
"Cardan angles") can be found for example at:
[Euler Angles](https://en.wikipedia.org/wiki/Euler_angles).
Following above article when refering to Euler/Cardan angles we will use "Euler angles" term, with Cardan angles as
a type of Euler angles.

#### Convention

Throughut the CRF the convention for writing Euler/Cardan angles representation is that,
for example, CardanXYZ means rotation:
\f$R_Z \circ R_Y \circ R_X\f$, so the order "X-Y-Z" is interpreted as the order in which
rotations are applied to a potential vector. Note that in the Eigen library this
convention is sometimes different. Details of this are described
in [Conversions](@ref group_rotation_conversions).

Warning: In Matlab, and several libraries in python this convention is different

\anchor cardanXYZ_representation

#### CardanXYZ

Represents rotation as a three Cardan angles, corresponding to rotating, respectively, around
X, Y, Z.

\anchor eulerZXZ_representation

#### EulerZXZ

Represents rotation as a three Cardan angles, corresponding to rotating, respectively, around
Z, X, Z.


#### Classes implementation

Details about the implementation of the
[EulerAngles](@ref crf::math::rotation::EulerAngles),
[CardanXYZ](@ref crf::math::rotation::CardanXYZ),
[EulerZXZ](@ref crf::math::rotation::EulerZXZ)
classes can be found in [EulerAngles](@ref group_rotation_euler_angles).

### Alternative names

There is an alias to call
[RotationRepresentation](@ref crf::math::rotation::RotationRepresentation)
an [OrientationRepresentation](@ref crf::math::rotation::OrientationRepresentation)
when it is more suitable within the context.
