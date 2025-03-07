@ingroup group_rotation_comparison

### Quaternion

The quaternions are compared as the rotations they represent, so for all quaternion
\f$q\f$, quaternions \f$q\f$ and \f$-q\f$ are treated as equal.

The parameter _accuracy_ can be used to adjust floating point values should be, to be considered
equal. Quaternions are compared with requirement to be equal up to this accuracy
on each coordinate (up to simultanious change of sign on all coordinates, as
noted above).

### Matrix

Matrices are compared coordinate by coordinate. On each coordinate the difference must
be smaller than a given parameter 'accuracy'.

### AngleAxis

AngleAxis values are compared **not** as the rotations they represent, but
by they coordinate-to-coordinate values, up to a given accuracy. For example the following angle axes:
```
Angle:
0.1000000000000000
Axis:
x: 1.0000000000000000
y: 0.0000000000000000
z: 0.0000000000000000
```
```
Angle:
3.241592653589793
Axis:
x: 1.0000000000000000
y: 0.0000000000000000
z: 0.0000000000000000
```
```
Angle:
-0.1000000000000000
Axis:
x: -1.0000000000000000
y:  0.0000000000000000
z:  0.0000000000000000
```
will be all considered pairwise different, despite them all representing the same rotation.
The choice for that was made, as these representations are used in CRF usually when such
distinction matter. AngleAxis can always be compared as a rotations they represent by comparing
[Rotation](@ref crf::math::rotation::Rotation) objects constructed from them.

During comparison, the axis is scaled by the angle, to maintain the proper accuracy restrictions
for small and large angles (accuracy is less restrictive in the differences of the axes
ib small angles and more restrictive for large angles). This scaling provides better fitting
on the accuracy scale (which presentations are considered the same) in relation to oder
representation (which means that with this scaling results from comparison are more or
less the same in the angleAxis representation (there are always some small discrepancies)
as in other representation. Without this scaling results would be significantly different.).

### CardanXYZ/EulerZXZ

CardanXYZ and EulerZXZ classes are compared **not** as the rotations they represent, but
by they coordinate-to-coordinate values, up to a given accuracy.
For example the following cardan XYZ angles:
```
CardanXYZ:
X: -3.141592653589793
Y: -3.141592653589793
Z: -3.041592653589793
```
```
CardanXYZ:
X: 0.0000000000000000
Y: 0.0000000000000000
Z: 0.1000000000000000
```
will be considered different, regardless of them representing the same rotation.
The choice for that was made, as these representations are used in CRF usually when such
distinction matter. CardanXYZ/EulerZXZ can always be compared as a rotations they represent by
comparing [Rotation](@ref crf::math::rotation::Rotation) objects constructed from them.

### Rotation

Comparison is always done in quaternion representation. The quaternions are compared as the
rotations they represent, so for all quaternion \f$q\f$, quaternions \f$q\f$ and \f$-q\f$ are
treated as equal.

Comparison is done 'as the rotations' which means that all additional data described in the section
_Additional data in some representations_ of [RotationRepresentation](@ref group_rotation_representations)
is discarded. For example the cardan XYZ angles:
```
CardanXYZ:
X: -3.141592653589793
Y: -3.141592653589793
Z: -3.041592653589793
```
```
CardanXYZ:
X: 0.0000000000000000
Y: 0.0000000000000000
Z: 0.1000000000000000
```
will be considered the same, as they represent the same rotation.

If there is a need to compare the numerical values in some representation not as a rotation it
represents, but by the bare values it is always possible by comparing via function areAlmostEqual()
values got from getters directltly.

The parameter _accuracy_ can be used to adjust floating point values should be, to be considered
equal. Quaternions are compared with requirement to be equal up to this accuracy on each coordinate
(up to simultanious change of sign on all coordinates, as noted above).
