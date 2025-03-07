@ingroup group_mass_spring_damper_rn

This module obtain the displacement in n dimensions, from the force applied as an input. In this case, instead of having scalar values, we will treat matrix variables. In this way, the equation of the model will be:

\f[ F = M \ddot{X} + D \dot{X} + KX \f]

For example, for the force, we will have the following matrix:

\f[
F = \begin{bmatrix}
    f_{11} & f_{12} & \cdots & f_{1n} \\
    f_{21} & f_{22} & \cdots & f_{12} \\
    \vdots & \vdots & \ddots & \vdots \\
    f_{n1} & f_{n2} & \cdots & f_{nn}
\end{bmatrix}
\f]

### Decoupled system
In a decoupled system, every component works independently, without affecting the rest, i.e., the matrix are diagonal:

\f[
F = \begin{bmatrix}
    f_{1} & 0 & \cdots & 0 \\
    0 & f_{2} & \cdots & 0 \\
    \vdots & \vdots & \ddots & \vdots \\
    0 & 0 & \cdots & f_{n}
\end{bmatrix}, \quad
X = \begin{bmatrix}
    x_{1} & 0 & \cdots & 0 \\
    0 & x_{2} & \cdots & 0 \\
    \vdots & \vdots & \ddots & \vdots \\
    0 & 0 & \cdots & x_{n}
\end{bmatrix}
\f]

\f[
M = \begin{bmatrix}
    m_{1} & 0 & \cdots & 0 \\
    0 & m_{2} & \cdots & 0 \\
    \vdots & \vdots & \ddots & \vdots \\
    0 & 0 & \cdots & m_{n}
\end{bmatrix}, \quad
D = \begin{bmatrix}
    d_{1} & 0 & \cdots & 0 \\
    0 & d_{2} & \cdots & 0 \\
    \vdots & \vdots & \ddots & \vdots \\
    0 & 0 & \cdots & d_{n}
\end{bmatrix}, \quad
K = \begin{bmatrix}
    k_{1} & 0 & \cdots & 0 \\
    0 & k_{2} & \cdots & 0 \\
    \vdots & \vdots & \ddots & \vdots \\
    0 & 0 & \cdots & k_{n}
\end{bmatrix}
\f]

It is an enormous advantage, since we just need to use the module crf::math::massspringdamper::IMassSpringDamperR1 \f$ n \f$ times, since we have \f$ n \f$ independant differential equations:

\f[
    f_1(t) = m_1\ddot{x}_1(t) + d_1\dot{x}_1(t) + k_1x_1(t) \\
    f_2(t) = m_2\ddot{x}_2(t) + d_2\dot{x}_2(t) + k_2x_2(t) \\
    \vdots \\
    f_n(t) = m_n\ddot{x}_n(t) + d_n\dot{x}_n(t) + k_nx_n(t)
\f]

### General system
In a general system, one component could affect others and vice-versa, so we can't proceed just calling the crf::math::massspringdamper::IMassSpringDamperR1 \f$ n \f$ times as in decoupled systems.
