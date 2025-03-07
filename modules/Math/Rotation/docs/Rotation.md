@ingroup group_rotation

This module is a intended way of handling spacial rotations in the CRF.
It consists of:

- [Rotation](@ref crf::math::rotation::Rotation) class that represents the spacial rotation,
- [Arithmetic functions](@ref group_rotation_arithmetic) for composing and
inverting rotations,
- [Comparison functions](@ref group_rotation_comparison) for checking if rotations are equal,
- [Conversions functions](@ref group_rotation_conversions)
between different rotation representations,
- Classes for different [Euler/Cardan angles](@ref group_rotation_euler_angles)
representations,
- Functions for [validation](@ref group_is_valid_rotation) if given quaternion or
matrix describes a rotation,
- [JSON converters](@ref group_json_converters) for serialising and deserialising
[Rotation](@ref crf::math::rotation::Rotation) class,
- Enum class [RotationRepresentation](@ref crf::math::rotation::RotationRepresentation)
as a source of truth for which representations are supported,

Descriptions of different rotation representations used in the CRF can be found
[here](@ref group_rotation_representations).

