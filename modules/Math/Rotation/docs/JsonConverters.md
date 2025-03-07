@ingroup group_rotation_json_converters

Warning:
As Rotation module do not support infinite values, so do not the jsonConverters for this module.

### CardanXYZ

Serialisarion have always three fields, named after the axes:
```json
{
    "X": 1.4,
    "Y": 1.0,
    "Z": 3.1
}
```

### EulerZXZ

Serialisarion have always three fields, named after the axes:
```json
{
    "Z1": 1.1471123464639490,
    "X": -1.6415867259093058,
    "Z2": 0.1139727950424842
}
```

### Rotation class

To reduce the risk of mistakes, adopted convention for serialising
[Rotation](@ref crf::math::rotation::Rotation) class is
very verbose.
Json of the rotation always have only one field, with the name of the representation
that was used.

Note: These are the serialisations of the whole [Rotation](@ref crf::math::rotation::Rotation)
class. The representation choosen for the serialisation depend on the
representation used in the constructor or the last used setter. 

#### Quaternion representation

Serialisation of the quaternion representation:
```json
{
    "Quaternion": {
        "W": 0.5505666516902112,
        "X": -0.6362152372281484,
        "Y": 0.3613804315900029,
        "Z": 0.4018839604029799
    }
}
```

#### Matrix representation

Serialisation of the matrix representation:
```json
{
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
```

#### AngleAxis representation

Serialisation of the angleAxis representation:
```json
{
    "AngleAxis": {
        "Angle": 1.9755068924749106,
        "Axis": [
            -0.7621249848254679,
            0.4328991822668132,
            0.4814185346426796
        ]
    }
}
```

#### CardanXYZ representation

Serialisation of the cardanXYZ representation:
```json
{
    "CardanXYZ": {
        "X": 1.4,
        "Y": 1.0,
        "Z": 3.1
    }
}
```

#### EulerZXZ representation

Serialisation of the eulerZXZ representation:
```json
{
    "EulerZXZ": {
        "Z1": 1.1471123464639490,
        "X": -1.6415867259093058,
        "Z2": 0.1139727950424842
    }
}
```
