@ingroup group_rotation_rotation_class

[Rotation](@ref crf::math::rotation::Rotation) is an intended class to use in the CRF
as the representation of a rotation or an orientation in a three dimensional space.

### User's guide

[Rotation](@ref crf::math::rotation::Rotation) class supports five different representations of
rotations in regards of being constructed from them, returning them, as well as the conversions
between them:
    - [Quaternion](group__group__rotation__representations.html#quaternion_representation)
    - [Matrix](group__group__rotation__representations.html#matrix_representation)
    - [AngleAxis](group__group__rotation__representations.html#angleAxis_representation)
    - [CardanXYZ](group__group__rotation__representations.html#cardanXYZ_representation)
    - [EulerZXZ](group__group__rotation__representations.html#eulerZXZ_representation)

It can be constructed from either of them and then any represention can by get from
[Rotation](https://www.youtube.com/watch?v=dQw4w9WgXcQ) via its getters, regardles of the representation
used in the constructor.

#### Construction

During construction with a particular representation, the constructed
[Rotation](@ref crf::math::rotation::Rotation) object will be assigned with the corresponding
[RotationReresentation](@ref crf::math::rotation::RotationRepresentation) object. Method
[Rotation::getRepresentation()](@ref crf::math::rotation::Rotation::getRepresentation())
can be use to check this representation.

When using quaternion or matrix, input parameters are checked if they are,
respectively, a unitary quaternion or a rotation matrix up to a specified accuracy.
More about the details about these checks can be found at
[Is valid rotation](@ref group_rotation_is_valid_rotation).

#### Representation

The representation correspond to the rotation representation that was used

From the user perspective, in most usecases the only relevant difference in having different
[RotationReresentation](@ref crf::math::rotation::RotationRepresentation) of
the [Rotation](@ref crf::math::rotation::Rotation) object
will be behaviour of the printing operator '<<'.

In usecases that the exceptionally demanding and time strict tasks another difference that may
be relevant is a small performace difference. For example when constructing
[Rotation](@ref crf::math::rotation::Rotation) with one representation and then repedetely
using getters to get another ot between the reresentations in general.
Caution is advised not to optimise blindly based on this possibility and sacrifise readability too
recklesly, but first check if this phenomenon *really* would make a difference in a given usecase,
as it is suspected that it should almost never affect performance to a significant degree.

#### Setters

When using quaternion or matrix, input parameters are checked if they are,
respectively, a unitary quaternion or a rotation matrix up to a specified accuracy.
More about the details about these checks can be found at
[Is valid rotation](@ref group_rotation_is_valid_rotation).

#### Getters

Getters will give desired representation regardless of the representation provided in the
constructor or the last used setter. User should be aware of the phenomena described in
[Rotation representations#Additional data in some representations]
(group__group__rotation__representations.html#additional_data).

#### Arithmetic operations

For the information on the arithmetic operations on [Rotation](@ref crf::math::rotation::Rotation),
see [Rotation arithmetic](@ref group_rotation_arithmetic).

#### Alternative names

There is an alias to call [Rotation](@ref crf::math::rotation::Rotation) an
[Orientation](@ref crf::math::rotation::Orientation), when it is more suitable wthin the context.

#### Stream operator

Stream operator works accordingly to the representation provided in the constructor or the
last used setter.
Formats for different representation are as follows:
    - [Quaternion](group__group__rotation__representations.html#quaternion):
```txt
Quaternion:
w:  0.550566651690211
x: -0.636215237228148
y:  0.361380431590003
z:  0.401883960402980
```
    - [Matrix](group__group__rotation__representations.html#matrix_representation):
```txt
Matrix:
 0.415786932069279 -0.902359286921429 -0.113441369998196
-0.017303661133149 -0.132561091420906  0.991023783949047
-0.909297426825682 -0.410091787710933 -0.070731288834892
```
    - [AngleAxis](group__group__rotation__representations.html#angleAxis_representation):
```txt
Angle:
1.975506892474911
Axis:
-0.762124984825468
 0.432899182266813
 0.481418534642680
```
    - [CardanXYZ](group__group__rotation__representations.html#cardanXYZ_representation)
```txt
CardanXYZ:
X: 1.400000000000000
Y: 2.000000000000000
Z: 3.100000000000000
```
    - [EulerZXZ](group__group__rotation__representations.html#eulerZXZ_representation)
```txt
EulerZXZ:
Z:  1.147112346463949
X: -1.641586725909306
Z:  0.113972795042484
```

#### Additional remarks

Inputs should not have infinite values.

### Developer's guide

#### Representation

Internaly in the class rotation is stored using either:
    - [Eigen::Quaterniond](https://eigen.tuxfamily.org/dox/classEigen_1_1Quaternion.html),
    - [Eigen::Matrix3d](https://eigen.tuxfamily.org/dox/group__matrixtypedefs.html#ga84e9fd068879d808012bb6d5dbfecb17)
    - [Eigen::AngleAxisd](https://eigen.tuxfamily.org/dox/classEigen_1_1AngleAxis.html)
    - [CardanXYZ](@ref crf::math::rotation::CardanXYZ)
    - [EulerZXZ](@ref crf::math::rotation::EulerZXZ).

Note -- As described in
[Rotation representations](@ref group_rotation_representations)
AngleAxis, CardanXYZ and EulerZXZ representations sometimes carry more information
than quaternion or matrix representation.

The approach taken was to minimise redundant convertions, while keeping the structure simple.
There are still areas for improvement as described in
[Handling representations with caching](#Handling representations with caching).

#### Areas for improvement

Areas for improvement:

- handling different representations

##### Handling representations with caching

The main aspect that could be improved in the class is reducing the number of redundant
conversions by caching.

When the result is computed at the getter it could be stored in a corresponding member field
and the flag could be raised.

Then in the setter all flags woulg go down except of the one corresponding to the setted
representation.

However, it isn't easy to fully optimize this approach for the getter. In this case, if the desired
representation has not been computed yet, it is essential to determine from which available representation
to calculate it.

The simple approuch would be to use the RotationRepresentation mechanism (that would be kept
anyway for operator<<, also with a possible improvement to allow setting the Representation) and
write the logic, such that the representation set there is always valid and use it.

The most elaborate one is to determine which conversions are the fastest and then for every
representation have an order of representations in which they should be checked whether they
are already calculated.
