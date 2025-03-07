@ingroup group_rotation_euler_angles

### User's guide

### EulerAngles class

[EulerAngles](@ref crf::math::rotation::EulerAnfles) is an intended class to use in the CRF
as to inherit from when writing the class representing particular Euler angles.

Warning: These classes are ment to only strongly type different Euler angles. They do not
provide any functionality regarding e.g. resolving the singularities on their own.

### CardanXYZ

Inheriting from [EulerAngles](@ref crf::math::rotation::EulerAnfles),
[CardanXYZ](@ref crf::math::rotation::CardanXYZ) is an intended class to use in the CRF
to represent cardan XYZ angles.

The display string produced
via stream operator is in the format:

```
X: *angle*
Y: *angle*
Z: *angle*
```

For example for the CardanXYZ object with angles [0.0, 1.57, 3.14] it will be:

```
X: 0.0
Y: 1.57
Z: 3.14
```

### EulerZXZ

Inheriting from [EulerAngles](@ref crf::math::rotation::EulerAnfles),
[EulerZXZ](@ref crf::math::rotation::EulerZXZ) is an intended class to use in the CRF
to represent euler ZXZ angles.

The display string produced
via stream operator is in the format:

```
Z1: *angle*
X: *angle*
Z2: *angle*
```

For example for the EulerZXZ object with angles [0.0, 1.57, 3.14] it will be:

```
Z: 0.0
X: 1.57
Z: 3.14
```

### Developers guide

#### Future developements

Provide the functionality for resolving the singularities.
