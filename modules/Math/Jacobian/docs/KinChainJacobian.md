@ingroup group_kinchain_jacobian

Provided kinematic chain the Jacobian for the arm is calculated as:

\f[
\mathcal{J_a} =
\begin{bmatrix}
{_\mathcal{I}\textbf{n}_1} \times {_\mathcal{I}\textbf{r}_{1}} & {_\mathcal{I}\textbf{n}_2} & \cdots & {_\mathcal{I}\textbf{n}_{k}} \times {_\mathcal{I}\textbf{r}_{k}} \\
{_\mathcal{I}\textbf{n}_1} & 0 & \cdots & {_\mathcal{I}\textbf{n}_{k}}
\end{bmatrix},
\f]

where:
    - \f${_\mathcal{I}\textbf{n}_i}\f$ is the axis of the \f$i\f$-th joint,
    - \f${_\mathcal{I}\textbf{r}_i}\f$ is the vecotr from th \f$i\f$-th joint to the end effector.
    - Each column correspond to one joint. In the example above the second column correspond to the prismatc joint,
all others to the revolute joints. Value for the column for the revolute joint is:
        - cross product of the normalised axis of the joint and the vector between the joint
        and the end effector point as the linear part
        - the normalised axis of the joint as the angular part
    Value for the column for the prismatic joint is:
        - the normalised axis of the joint as the linear part
        - 0 as the angular part
    - First three rows correspond to the linear task velocities of the end effector point associated with consecutive joints.
    - Last three rows correspond to the angular task velocities of the end effector point associated with consecutive joints.

Jacobian for the mobile platform is calculated as:

\f[
\mathcal{J_p} =
\begin{bmatrix}
1 & 1 & 1 & 1 \\
1 & -1 & 1 & -1 \\
0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 \\
|w| & |w| & |w| & |w|
\end{bmatrix},
\f]

where
\f[
    w = \frac{platformLength + platformWidth}{2}.
\f]

**Note**: This form of the Jacobian for the platform is only valid, when the
platform was defined as required by the CRF standards, for example, in the case of URDF
see: [URDF's guideline](https://confluence.cern.ch/pages/viewpage.action?pageId=422151198)
Agreement with this standards is checked at the level of the KinematicChain class.

