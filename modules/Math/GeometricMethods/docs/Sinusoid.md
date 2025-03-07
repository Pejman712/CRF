@ingroup group_sinusoid

### Base Profile sin2 in the 2nd derivative

To provide smooth motor signals, we will use a trajectory profile which is four times (up to jerk) continuously differentiable and thus belongs to the C4 class, being the velocity the 0th derivative. If the 0th derivative is the position, it
would belong to the C3 class. First, a base profile ```σ(t)``` will be defined on ```t ∈ [0, tE ]``` such that ```σ(t) ∈ [0, 1]``` and ```σ(t)``` is strictly increasing. Applying a sin2 satisfies all requirements and can be piecewise defined in the 2nd derivative as:

\f[
    \label{eq:2ndDerivative}
	\ddot{\sigma} = \begin{cases}
        0                         & t<0\;
    	d''_{max} \sin(\omega t)^2  & t<\frac{\pi}{\omega}\:
		-d''_{max} \sin(\omega t)^2 & t<\frac{2\pi}{\omega}\,
		0                         & \mathrm{else},
	\end{cases}
\f]

with the maximum 2nd derivative ```d′′ max``` and angular 0th derivative ω. Since we are interested in limiting the dynamics of our trajectories to the physical limit of a certain motor, which is the maximum torque and thus corresponds to a maximum acceleration in our trajectory (assuming non-elastic gears), that is the maximum 1st derivative ```d′ max```. This means that the maximum 2nd derivative in (1) has to be written as ```d′′ max = f (d′ max)```. Integration of (4.11) leads to the 1st derivative.

\f[
	\label{eq:1stDerivative}
	\ddot{\sigma}(t) = \begin{cases}
		0                         & t<0\\
		-\frac{d''_{max}}{2 \omega} \left(\cos(\omega t) \sin(\omega t) - \omega t\right)  & t<\frac{\pi}{\omega}\\
		\frac{d''_{max}}{2 \omega} \left(\cos(\omega t) \sin(\omega t) - \omega t\right) + \sqrt{2 d''_{max}} & t<\frac{2\pi}{\omega}\\
		0                         & \mathrm{else}
	\end{cases}
\f]

and setting the maximum 1st derivative  ̇```σ(t = πω ) != d′``` max yields

\f[
    \label{eq:max1stDerivative}
	d''_{max} = \frac{2  \omega}{\pi}  d'_{max}.
\f]

Now the base profile needs to be normalized to satisfy  ̇```σ(t) ∈ [0, 1]```.  Integration of (4.12) leads to:

\f[
    \label{eq:0thDerivative}
	\sigma(t) = \begin{cases}
        0                         & t<0\\
		\frac{d''_{max}}{4 \omega^2} \left(\cos(\omega t)^2 + \omega^2 t^2 -1 \right)  & t<\frac{\pi}{\omega}\\
		-\frac{d''_{max}}{4 \omega^2} \left(\cos(\omega t)^2 + \omega^2 t^2 -1 \right)  + \sqrt{2 d''_{max}} t - \frac{d''_{max} \pi^2}{2 \omega^2} & t<\frac{2\pi}{\omega}\\
		1                         & \mathrm{else}
	\end{cases}
\f]


and setting ```σ(t = π ω ) ! = 1 2``` leads to:

\f[
    \label{eq:angular0thDerivative}
	\omega = \pi \frac{d''_{max}}{2}.
\f]

By combining equations (4.13) and (4.15) we find the final laws for the unknown variables in (4.11)

\f[
    \begin{split}
        d''_{max} &= 2 d'^2_{max} \\
        \omega &= \pi d'_{max}.
    \end{split}
\f]

There is also the possibility of computing ```d′′ max``` and ```ω``` based on ```tE```

\f[
    \begin{split}
        d''_{max} &= 2 \left( \frac{\omega}{\pi} \right)^2 \\
        \omega &= \frac{2 \pi}{t_E} .
    \end{split}
\f]

### Using Base Profile to Generate Trajectory

The base profile can now be used to provide a smooth guidance of a motor from any start velocity ```vS``` to any end velocity ```vE``` , considering the velocity the 0th derivative, with the maximum acceleration ```amax = d′ max``` like

\f[
	v(t) = v_S + \sigma(t) \Delta v,
\f]

with ```∆v = vE − vS```.
A plot with the results of computing the signals with maximum acceleration to 1.2 m/s is shown at Figure 4.1.

### Implementation in the CERN Robotic Framework - CRF
The implementation of this curve generator follows (inherits) the interface IGeometricMethods. It is located in /modules/Math/GeometricMethods. The behavior of each method is shown in their description.