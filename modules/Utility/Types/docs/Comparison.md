@ingroup group_types_comparison

All types exept TaskPose can have infinite values while being compared
and comparison takes this into account.

Comparisons are weak inequalities, which means that an object compared to itself
safisfies all:
    - 'areAlmostEqual
    - 'isBetween'
    - 'isLesser'
    - 'isGreater'

This is important e.g. in cases of
```cpp
std::numeric_limits<double>::infinity()
```
and
```cpp
-std::numeric_limits<double>::infinity()
```

### AreAlmostEqual

The intended function to compare objects of a partcular type, to check if they are the same
up to a given accuracy.

#### TaskPose

First the positions are compared on the actve coordinates (as specified by 'taskSpace' argument),
coordinate by coordinate if they are equal up to the parameter accuracy.

Then, dependent on the number of active angular coordinates:
 - If there are two or three angular coordinates active, then orientations are compared via 'areAlmostEqual' function for orientations (for the [Rotation](@ref crf::math::rotation::Rotation) class). The comparison is the same, regardles of whether two or three coordinates are active. This is because rotations around two directions spans already whole rotation group.
 - If there is only one angular coordinate active, then rotations are compared only on the coresponding coordinate from the cardanXYZ representation.
 - If there are no angular coordinates active, nothing from the angular part is compared and the result depends only on the linear part.

#### VectorXd and classes inheriting from it

Arguments are compared coordinate by coordinate, if they are equal up to the parameter accuracy.

#### Vector6d and classes inheriting from it

Arguments are compared coordinate by coordinate on the actve coordinates
(as specified by 'taskSpace' argument), if they are equal up to the parameter accuracy.

### IsBetween, IsLesser, IsGreater

The intended function to compare objects of a partcular type, to check if they are lesser
or greater than the other.

Ordering of the arguments.
For arguments 'lowerBound', 'upperBound', 'vector', we have that 'IsBetween' checks
whether lowerBound <= vector <= upperBound coordinate wise.

For arguments 'vector1', 'vector2', we have that 'IsLesser' checks
whether vector1 <= vector2 coordinate wise.

For arguments 'vector1', 'vector2', we have that 'IsGreater' checks
whether vector1 >= vector2 coordinate wise.

Note about the NaNs:
comparisons are written in a way, that whenever there is a nan in either of compared objects,
on the coordinate that is not deactivated, comparison returns false.

Because of that, it is needed to:

Always make direct comparisons to check the boundaries as vel < maxVel
not as !(vel > maxvel) as in case of NaNs the comparison always return false.
Then vel < maxVel, comparison will return false and the code will proceed as if the boundary
won't be satisfied.
On the other hand in the case of !(vel > maxvel), even if the boundary won't be satisfied, because
of NaN, compartson vel > maxvel will return false, so the whole !(vel > maxvel) will return true
and the code would proceed as if the bondary would be satisfied.

#### VectorXd and classes inheriting from it

Arguments are compared coordinate by coordinate, if they satisfy given inequalities.

#### Vector6d and classes inheriting from it

Arguments are compared coordinate by coordinate on the actve coordinates
(as specified by 'taskSpace' argument), if they satisfy given inequalities.

### Developers guide

to ensure that the comparisons return false when at least one of the active coordinates of
the arguments is equal to NaN, they are written in the way (on the example of 'isLesser'):
```cpp
if (mask[i] && !(vaector1[i] >= vector2[i])) {
    return false;
}
```
This way, as comparison of the NaN always returns false, the function will return false, whenever
there is a NaN present, while maintaining the usual semantic in the absense of the NaN.

#### Possible future impovements

Add support of infinite values in the TaskPose
