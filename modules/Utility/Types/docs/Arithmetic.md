@ingroup group_types_arithmetic

### Multiply

Multiplication is the multiplication as in homogeneous transformation matrices.

For the task poses \f$p_1\f$ and \f$p_2\f$, where \f$p_1 = (t_1,\ R_1)\f$ consists of
the position \f$t_1\f$ and orientation \f$R_1\f$ and \f$p_2 = (t_2,\ R_2)\f$ consists of
the position \f$t_2\f$ and orientation \f$R_2\f$, we have that:

\f[
p_1p_2 = (t_1,\ R_1)(t_2,\ R_2) = (t_1 + R_1t_2,\ R_1R_2).
\f]

So the pose \f$p_1p_2\f$ consists of the position \f$t_1 + R_1t_2\f$ and the orientation
\f$R_1R_2\f$.

Orientations are multiplied as a [Rotation](@ref crf::math::rotation::Rotation) objects.

### Invert

Inversion is the inversion as in homogeneous transformation matrices.

For the task pose \f$p\f$, where \f$p = (t,\ R)\f$ consists of
the position \f$t\f$ and orientation \f$R\f$, we have that:

\f[
p^{-1} = (t,\ R)^{-1} = (-R^{-1}t,\ R^{-1}).
\f]

So the pose \f$p^{-1}\f$ consists of the position \f$-R^{-1}t\f$ and the orientation
\f$R^{-1}\f$.

Orientation is inverted as a [Rotation](@ref crf::math::rotation::Rotation) object.
