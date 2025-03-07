@ingroup group_rotation_arithmetic

These are intended functions to use in the CRF for performing arithmetical operations on Rotations.

As described in the section _Additional data in some representations_ of [RotationRepresentation](@ref group_rotation_representations) AngleAxis, CardanXYZ and EulerZXZ representations may carry some additional information apart from
describing a particular rotation. There is however no straightforward way of keeping this
information when multiplying rotations. Because of that, when multiplying all this information is
lost. For consistency we are droping this information also in all the other arithmetical operations.

The result of arithmetical operations is always returned in quaternion representation.

### Multiply

This is intended function to use to compose rotations in the CRF.

For arguments representing rotations \f$R_1\f$ and \f$R_2\f$,
function [Multiply](@ref crf::math::rotation::multiply())
multiplies two quaternions representing these rotations and returns
[Rotation](@ref crf::math::rotation::Rotation) object representing the result \f$R_1*R_2\f$
in the quaternion representation.

### Invert

This is intended function to use to compose rotations in the CRF.

For argument representing rotation \f$R\f$,
function [Invert](@ref crf::math::rotation::invert())
inverts quaternion representing this rotation and returns
[Rotation](@ref crf::math::rotation::Rotation) object representing the result \f$R^{-1}\f$
in the quaternion representation.

The [Invert](@ref crf::math::rotation::invert()) function
returns a new [Rotation](@ref crf::math::rotation::Rotation) object, it is **not** done
"in place".

### AngularVelocityFromRotation

For argument representing rotation \f$R\f$
function [AngularVelocityFromRotation](@ref crf::math::rotation::angularVelocityFromRotation())
calculates angular velocity, traveling with wich will go from identity rotation
to \f$R\f$ in time 1 and returns an _Eigen::Vector3d_ object representing the result.

### RotationFromAngularVelocity

For argument representing angulat velocity \f$\omega\f$
function [RotationFromAngularVelocity](@ref crf::math::rotation::rotationFromAngularVelocity())
calculates rotation to which traveling with \f$\omega\f$ will go
in time 1 and returns 
[Rotation](@ref crf::math::rotation::Rotation) object representing the result
in the angleAxis representation.

The parameter _accuracy_ represents the minimal norm of the angular velocity, below which
it will be treated as a zero vector.
