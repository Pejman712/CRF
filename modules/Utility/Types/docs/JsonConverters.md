@ingroup group_types_json_converters

All of our types are able to represent infinity, negative infinity and NaN (not a number) values.
This is particulary usefull in cases where there are no limits so inifinity can be an option for example in robotic joints that can turn unlimited.
Or when some dimensions are non used, as in the case in reduced task space, then the placeholder
values can be represented as NaNs.

This would be represented in the vector as:

```json
{
    "MaxPosition" : [0, "inf", 0, "NaN", "-inf", 0]
}
```
This example would represent infinity, NaN and negative infinity respectively.

More on the convertions of these special values in
[StdJsonConverters](@ref group_std_json_converters).

### Conventions

This is the documentation on how the types inside the CRF are parsed into and from JSONs.

### Joints Types

Jsons of the joint types consists only of the list.

#### JointPositions

Serialisation of the JointPositions:
```json
[
    -1.2452654789023454,
    "-inf",
    2.8456356344354765,
    4.0,
    "inf",
    0.0001231242,
    5.22784
]
```

#### JointVelocities

Serialisation of the JointVelocities:
```json
[
    -1.2452654789023454,
    "-inf",
    2.8456356344354765,
    4.0,
    "inf",
    0.0001231242,
    5.22784
]

```

#### JointAccelerations

Serialisation of the JointAccelerations:
```json
[
    -1.2452654789023454,
    "-inf",
    2.8456356344354765,
    4.0,
    "inf",
    0.0001231242,
    5.22784
]

```

#### JointForceTorques

Serialisation of the JointForceTorques:
```json
[
    -1.2452654789023454,
    "-inf",
    2.8456356344354765,
    4.0,
    "inf",
    0.0001231242,
    5.22784
]
```

### Task Types

#### TaskPose

Json of the TaskPose always have exactly two fields -- one for position
and one for orientation.
The field for position is a three dimensional vector, the field for rotation
contains a json of [Rotation](@ref crf::math::rotation::Rotation), as described in
[Rotation JsonConverters](@ref group_rotation_json_converters).

The representation choosen for the serialisation depend on the
representation used in the constructor or the last used setter. 

#### Quaternion representation

Serialisation of the quaternion representation:
```json
{
    "Position": [
        -2.537896,
        3.451273,
        -23.45126754
    ],
    "Orientation": {
        "Quaternion": {
            "W": 0.5505666516902112,
            "X": -0.6362152372281484,
            "Y": 0.3613804315900029,
            "Z": 0.4018839604029799
        }
    }
}
```

#### Matrix representation

Serialisation of the matrix representation:
```json
{
    "Position": [
        -2.537896,
        3.451273,
        -23.45126754
    ],
    "Orientation": {
        "Matrix": {
            "Row1": [
                0.4157869320692789,
                -0.9023592869214287,
                -0.1134413699981963
            ],
            "Row2": [
                -0.0173036611331485,
                -0.1325610914209059,
                0.9910237839490472
            ],
            "Row3": [
                -0.9092974268256818,
                -0.4100917877109334,
                -0.0707312888348917
            ]
        }
    }
}
```

#### AngleAxis representation

Serialisation of the angleAxis representation:
```json
{
    "Position": [
        -2.537896,
        3.451273,
        -23.45126754
    ],
    "Orientation": {
        "AngleAxis": {
            "Angle": 1.9755068924749106,
            "Axis": [
                -0.7621249848254679,
                0.4328991822668132,
                0.4814185346426796
            ]
        }
    }
}
```

#### CardanXYZ representation

Serialisation of the cardanXYZ representation:
```json
{
    "Position": [
        -2.537896,
        3.451273,
        -23.45126754
    ],
    "Orientation": {
        "CardanXYZ": {
            "X": 1.4,
            "Y": 1.0,
            "Z": 3.1
        }
    }
}
```

#### EulerZXZ representation

Serialisation of the eulerZXZ representation:
```json
{
    "Position": [
        -2.537896,
        3.451273,
        -23.45126754
    ],
    "Orientation": {
        "EulerZXZ": {
            "Z1": 1.1471123464639490,
            "X": -1.6415867259093058,
            "Z2": 0.1139727950424842
        }
    }
}
```

#### Linear task types

Jsons of the linear task types consists only of the list.

#### TaskVelocity

Serialisation of the TaskVelocity:
```json
[
    2.2,
    "-inf",
    4.7,
    "inf",
    4.7,
    3.9
]
```

#### TaskAcceleration

Serialisation of the TaskAcceleration:
```json
[
    2.2,
    "-inf",
    4.7,
    "inf",
    4.7,
    3.9
]
```

#### TaskForceTorque

Serialisation of the TaskForceToeque:
```json
[
    2.2,
    "-inf",
    4.7,
    "inf",
    4.7,
    3.9
]
```
