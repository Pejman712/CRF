@ingroup group_rotation_conversions

These are intended functions to use in the CRF for converting between different
representations of rotations.
They are used internally in the [Rotation](@ref crf::math::rotation::Rotation) class,
that supports conversions between different representations, but can also be used standalone.
When crf::math::rotation::Rotation class is used, every representation can be obtained from it,
regardless of its crf::math::rotation::RotationRepresentation.
However, these functions might still be usefull to use together with
[Rotation](@ref crf::math::rotation::Rotation) class to convert one representation,
to the another, compatible with a desired constructor setter. For details regarding this case see
[Rotation Class](@ref group_rotation_rotation_class).

It's important to note that when converting Euler Angles, singularities may not be accounted for. As a result, converting to a different representation and then back again may not give the same results. Furthermore, the conversion process may also lead to suboptimal presentations in certain representations, for example returning cardan XYZ angles:
```
CardanXYZ:
X: -3.141592653589793
Y: -3.141592653589793
Z: -3.041592653589793
```
instead of:
```
CardanXYZ:
X: 0.0000000000000000
Y: 0.0000000000000000
Z: 0.1000000000000000
```
, as the two presentations above represent the same rotation.

### Developer's guide

#### Euler/Cardan angles in Eigen library

The important detail as described in the
[rotation representations](@ref group_rotation_representations) convention for writing
Euler/Cardan angles is that, for example,
[CardanXYZ](@ref crf::math::rotation::CardanXYZ)
means rotation:
\f$R_Z \circ R_Y \circ R_X\f$, so the order "X-Y-Z" means the order in which rotations
are applied to a potential vector. However the function
[```eulerAngles```](https://www.eigen.tuxfamily.org/dox/group__Geometry__Module.html#ga17994d2e81b723295f5bc3b1f862ed3b)
from Eigen library works in such
manner, that to obtain representation in CRF called CardanXYZ, it needs to be called
with the arguments
[```eulerAngles(2, 1, 0)```](https://www.eigen.tuxfamily.org/dox/group__Geometry__Module.html#ga17994d2e81b723295f5bc3b1f862ed3b)
("2" always stands for "Z", "1" always stands for "Y",
"0" always stands for "X" in
[```eulerAngles```](https://www.eigen.tuxfamily.org/dox/group__Geometry__Module.html#ga17994d2e81b723295f5bc3b1f862ed3b)
arguments),
so there the order "2-1-0" means the order the rotations are writen, from left to right during
composition \f$R_Z \circ R_Y \circ R_X\f$.
The return value of
[```eulerAngles```](https://www.eigen.tuxfamily.org/dox/group__Geometry__Module.html#ga17994d2e81b723295f5bc3b1f862ed3b)
function is also in this convention,
so, the return of
[```eulerAngles(2, 1, 0)```](https://www.eigen.tuxfamily.org/dox/group__Geometry__Module.html#ga17994d2e81b723295f5bc3b1f862ed3b)
will be an
[```Eigen::Vector3d```](https://www.eigen.tuxfamily.org/dox/group__matrixtypedefs.html#gaabb0b4639dc0b48e691e02e95873b0f1)
with
cordinates corresponding to rotations around, accordingly, Z, Y and X.

To obtain
[CardanXYZ](@ref crf::math::rotation::CardanXYZ)
in CRF convention, the return vector
(let us call it "eigenCardan" in this example) of
[```eulerAngles(2, 1, 0)```](https://www.eigen.tuxfamily.org/dox/group__Geometry__Module.html#ga17994d2e81b723295f5bc3b1f862ed3b)
needs to be witten to
[CardanXYZ](@ref crf::math::rotation::CardanXYZ)
in opposite direction. In the pseudocode:

```pseudocode
cardanXYZ[0] = eigenCardan[2];
cardanXYZ[1] = eigenCardan[1];
cardanXYZ[2] = eigenCardan[0];
```

Note that this is **ABSOLUTELY NOT** equivalent to calling
[```eulerAngles(2, 1, 0)```](https://www.eigen.tuxfamily.org/dox/group__Geometry__Module.html#ga17994d2e81b723295f5bc3b1f862ed3b)
and then
writing it to CardanXYZ in the straightforward direction, as
[```eulerAngles(2, 1, 0)```](https://www.eigen.tuxfamily.org/dox/group__Geometry__Module.html#ga17994d2e81b723295f5bc3b1f862ed3b)
gives
three Cardan angles corresponding to composing rotations in order:
\f$R_X \circ R_Y \circ R_Z\f$, which are completely different (not only different by
reversing the order of Cardan angles) from Cardan angles corresponding to composing
rotations in order: \f$R_Z \circ R_Y \circ R_X\f$, which are the return of calling
[```eulerAngles(2, 1, 0)```](https://www.eigen.tuxfamily.org/dox/group__Geometry__Module.html#ga17994d2e81b723295f5bc3b1f862ed3b).

The same principles applies to Euler angles representation other than
[CardanXYZ](@ref crf::math::rotation::CardanXYZ).

#### Directions of further improvements

Taking into account singularities in Euler Angles
