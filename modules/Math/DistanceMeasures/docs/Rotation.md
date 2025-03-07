@ingroup group_distance_measures_rotation

Functions that represent some measures of distance between two rotations from \f$SO(3)\f$.
All of these measures returns a three dimensional vector.


**Note:** No of these meassures is an angular velocity traveling with which will go from
one rotation to another. For the function of this semantics, see
crf::math::rotation::angularVelocityFromRotation()

#### Quaternion

For the two rotations \f$R_1\f$ and \f$R_2\f$, representaed respectively by quaternions
\f$q_1 = w_1 + x_1i + y_1j + z_1k\f$ and \f$q_2 = w_2 + x_2i + y_2j + z_2k\f$,
we calculate distance measure as:
\f[
w_1\begin{pmatrix}x_2 \\ y_2 \\ z_2\end{pmatrix} -
w_2\begin{pmatrix}x_1 \\ y_1 \\ z_1\end{pmatrix} -
\begin{pmatrix}x_2 \\ y_2 \\ z_2\end{pmatrix} \times
\begin{pmatrix}x_1 \\ y_1 \\ z_1\end{pmatrix}.
\f]

Error calculated in a way such that asymptotically stable error dynamics in closed loop
can be proven with Lyapunov.

#### Rotation Matrix

For the two rotations \f$R_1\f$ and \f$R_2\f$, representaed respectively by rotation matrices
\f$M_1\f$ and \f$M_2\f$, whith column, respectively
\f$M_1 = \begin{pmatrix}c_1^1 & c_1^2 & c_1^3\end{pmatrix}\f$ and
\f$M_2 = \begin{pmatrix}c_2^1 & c_2^2 & c_2^3\end{pmatrix}\f$.
we calculate distance measure as:
\f[
0.5\left(c_1^1\times c_2^1 + c_1^2\times c_2^2 + c_1^3\times c_2^3\right)
\f]

#### CardanXYZ and EulerZXZ

These distance measures perform coordinatewise substraction.
Fir the two rotations \f$R_1\f$ and \f$R_2\f$, representaed respectively by euler angles
\f$\alpha_1, \beta_1, \gamma_1\f$ and \f$\alpha_2, \beta_2, \gamma_2\f$,
we calculate these measures as:
\f[
\begin{pmatrix}\alpha_2 - \alpha_1 \\ \beta_2 - \beta_1 \\ \gamma_2 - \gamma_1\end{pmatrix}.
\f]
