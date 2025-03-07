@ingroup group_mass_spring_damper_r1

This module obtain the displacement in one dimension, from the force applied as an input.

### Model

The mathematical model is constituted by the following parameters:

* Mass (m)
* Spring (k)
* Damper (d)

\f[ f(t) = m\ddot{x}(t) + d\dot{x}(t) + kx(t) \f]

With these parameters, we have the following transfer function:

$$ G(s) = \frac{1}{ms^2+ds+k} $$

### Direct Form II
This implementation uses internally the IIR Digital Filter, i.e., calls the method direct2FormFilter() to obtain the displacement.

In order to use this filter, first it is necessary to calculate the coeficientes \f$ a \f$ and \f$ b \f$ , i.e., to obtain \f$ G(z) \f$.

$$ G(z) = \frac{b_0 + b_1 \cdot z^{-1} + b_2 \cdot z^{-2}}{a_0 + a_1 \cdot z^{-1} +a_2 \cdot z^{-2}} $$

This method divide the process in two steps, first obtaining the intermediate signal \f$ u \f$ from the numerator and then the output \f$ y \f$ with the complete transfer function. Considering \f$ a_0 = 1 \f$ by convention, we have:

$$ U(z) = \frac{1}{1 + a_1 \cdot z^{-1} + a_2 \cdot z^{-2}} \cdot X(z) \Longrightarrow Y(z) = (b_0 + b_1 \cdot z^{-1} + b_2 \cdot z^{-2}) \cdot U(z)$$
$$Y(z) = G(z)\cdot X(z)$$

In order to transform \f$ s \f$ to \f$ z \f$ domain, we can follow three methods:

* Impulse Invariance Method (\ref group_iim_mass_spring_damper_r1 "IIM")
* Step Invariance Method (\ref group_sim_mass_spring_damper_r1 "SIM")
* Bilinear Transform Method (\ref group_btm_mass_spring_damper_r1 "BTM")

### Comparison

<table>
    <tr>
        <th>Method</th>
        <th>Advantages</th>
        <th>Disadvantages</th>
    </tr>
    <tr>
        <td>Impulse Invariance Method</td>
        <td>
            <ul>
                <li>Simplicity: direct mapping \f$ s-z \f$ through \f$ z = e^{sT} \f$ </li>
                <li>Linear relationship between analog and digital frecuencies.</li>
            </ul>
        </td>
        <td>
            <ul>
                <li>Aliasing as long as we increase the frecuency, especially when we surpass Nyquist frecuency.</li>
                <li>Continuous gain is not preserved.</li>
            </ul>
        </td>
    </tr>
    <tr>
        <td>Step Invariance Method</td>
        <td>
            <ul>
                <li>Better aproximation than IIM.</li>
                <li>Continuous gain is preserved.</li>
            </ul>
        </td>
        <td>Same aliasing problems as IIM, due to the transformation \f$ z = e^{sT} \f$.</td>
    </tr>
    <tr>
        <td>Bilinear Transform Method</td>
        <td>
            <ul>
                <li>No aliasing problems, as it accomplish a mapping from the infinite frecuencies from the \f$ s \f$ plane to the \f$ z \f$ plane.</li>
                <li>Stability is preserved, i.e., as long as the continuous poles are in the negative area of the \f$ s \f$ plane, they will be inside the unitary circle in the \f$ z \f$ plane.</li>
                <li>Thanks to realtionship with the tangent, continuous gain is preserved: \f$ \omega = 0 \rightarrow \Omega = 0\f$</li>
            </ul>
        </td>
        <td>
            <ul>
                <li>Non-inear relationship between analog and digital frecuencies: \f$ \Omega = \frac{2}{T}\tan\left(\frac{\omega}{2}\right) \f$ </li>
                <li>At high frequencies, the frequency response of the system can deviate significantly from that of the continuous system. To solve this problem, it will be necessary to apply a signal pre-warping process.</li>
            </ul>
        </td>
    </tr>
</table>

The different methods for MSD will be analyzed and compared for different contexts. The sampling time T = 0.01 for all the cases, except for the example of aliasing effect.

In order to analyze the behaviour of the models, we will use the following signals as inputs:
* Impulse
* Step
* Ramp
* Sine

#### Overdamped
An overdamped system does not have oscilations, i.e., its poles are on the real axis. Using \f$ m = 1, d = 3, k = 2 \f$ as parameters, we have the following transfer function:

$$ G(s) = \frac{1}{s^2 + 3s + 2} $$

<div style="margin: 0 auto; width: fit-content; max-width: 500px;">
    <img src="https://cernbox.cern.ch/remote.php/dav/public-files/qkjWiynayy2Q9Lu/overdamped_s_plane.png?scalingup=0&preview=1&a=1&c=94350939883831296%3A8a4af50f&x=3840&y=3840" alt="overdamped_s_plane">
</div>

The transfer functions \f$ G(z) \f$ for each method are:

$$ G_{BTM}(z) = 10^{-5} \frac{2.463 z^2 + 4.926 z + 2.463}{z^2 - 1.97z + 0.9704} $$
$$ G_{IIM}(z) = 10^{-5} \frac{9.851z}{z^2 - 1.97z + 0.9704} $$
$$ G_{SIM}(z) = 10^{-5} \frac{4.95 z + 4.901}{z^2 - 1.97z + 0.9704} $$

It can be observed that all of them have the same poles but different zeros, both in magnitude and number. This can be seen clearly in the \f$ z \f$ planes.

<div style="margin: 0 auto; width: fit-content; max-width: 800px;">
    <img src="https://cernbox.cern.ch/remote.php/dav/public-files/aUCvtNAohZn95cz/overdamped_z_plane.png?scalingup=0&preview=1&a=1&c=94350939615395840%3A32776391&x=3840&y=3840" alt="overdamped_z_plane">
</div>

Analyzing the time response, only an evolution can be observed, given by the same poles. Furthermore, the same magnitude is reached for the three methods, even though they have different zeros.

<div style="margin: 0 auto; width: fit-content; max-width: 800px;">
    <img src="https://cernbox.cern.ch/remote.php/dav/public-files/IAOPr9MeMLy8Tcd/overdamped_response_impulse_ramp.png?scalingup=0&preview=1&a=1&c=94350940420702208%3A2528fbab&x=3840&y=3840" alt="overdamped_response_impulse_ramp">
</div>

<div style="margin: 0 auto; width: fit-content; max-width: 800px;">
    <img src="https://cernbox.cern.ch/remote.php/dav/public-files/kdg2nTfTKkkONw0/overdamped_response_sine_step.png?scalingup=0&preview=1&a=1&c=94350940152266752%3A63f7eecd&x=3840&y=3840" alt="overdamped_response_sine_step">
</div>

If we now analyse the frequency response of the system, we can see that in general at low frequencies the magnitude is similar, although it begins to be different after the cut-off frequency. As for the phase, it is very similar for the three methods, except for SIM, where there is a new decrease in the last part.

<div style="margin: 0 auto; width: fit-content;">
    <img src="https://cernbox.cern.ch/remote.php/dav/public-files/OENQ1FBHHKAQ7S9/overdamped_bode_plot.png?scalingup=0&preview=1&a=1&c=94350940957573120%3Abc3cc0c2&x=3840&y=3840" alt="overdamped_bode_plot">
</div>

In conclusion, we have analyzed three different aspects compare the models obtained:

* Time response: similar evolution.
* Magnitude: similar value at low frecuencies.
* Phase: SIM different from IIM and BTM at high frecuencies.

#### Critically damped
A critically damped system has the poles on the boundary between real and complex values, i.e., just before any oscilation could appear. For this case, the following parameters \f$ m = 1, d = 4, k = 1 \f$ have been chosen, giving rise to a value equal to zero at the root of the poles equations.

$$ G(s) = \frac{1}{s^2 + 2s + 1} $$

<div style="margin: 0 auto; width: fit-content; max-width: 500px;">
    <img src="https://cernbox.cern.ch/remote.php/dav/public-files/BM3pBeXUK5KtT17/critically_damped_s_plane.png?scalingup=0&preview=1&a=1&c=94350941226008576%3A3023e53a&x=3840&y=3840" alt="critically_damped_s_plane">
</div>

The transfer functions \f$ G(z) \f$ for each method are:

$$ G_{BTM}(z) = 10^{-5} \frac{2.47519 z^2 + 4.95037 z + 2.47519}{z^2 - 1.9801z + 0.980199} $$
$$ G_{IIM}(z) = 10^{-5} \frac{9.9005z}{z^2 - 1.9801z + 0.980199} $$
$$ G_{SIM}(z) = 10^{-5} \frac{4.96679 z + 4.93379}{z^2 - 1.9801z + 0.980199} $$

As for the overdamped system, it can be observed that all of them have the same poles but different zeros, both in magnitude and number. This can be seen clearly in the \f$ z \f$ planes.

<div style="margin: 0 auto; width: fit-content; max-width: 800px;">
    <img src="https://cernbox.cern.ch/remote.php/dav/public-files/v9yaWhugY7uxDLP/critically_damped_z_plane.png?scalingup=0&preview=1&a=1&c=94350940689137664%3Ae585377c&x=3840&y=3840" alt="critically_damped_z_plane">
</div>

Analyzing the time response, only an evolution can be observed, given by the same poles. Furthermore, the same magnitude is reached for the three methods, even though they have different zeros.

<div style="margin: 0 auto; width: fit-content; max-width: 800px;">
    <img src="https://cernbox.cern.ch/remote.php/dav/public-files/PJRiPkaTu0kG29h/critically_damped_response_impulse_ramp.png?scalingup=0&preview=1&a=1&c=94350941762879488%3A37002e9b&x=3840&y=3840" alt="critically_damped_response_impulse_ramp">
</div>

<div style="margin: 0 auto; width: fit-content; max-width: 800px;">
    <img src="https://cernbox.cern.ch/remote.php/dav/public-files/6CX6pMV88sPCAKv/critically_damped_response_sine_step.png?scalingup=0&preview=1&a=1&c=94350941494444032%3Ab63eb25c&x=3840&y=3840" alt="critically_damped_response_sine_step">
</div>

If we now analyse the frequency response of the system, we can see that in general at low frequencies the magnitude is similar, although it begins to be different after the cut-off frequency. As for the phase, it is very similar for the three methods, except for SIM, where there is another decrease in the last part.

<div style="margin: 0 auto; width: fit-content;">
    <img src="https://cernbox.cern.ch/remote.php/dav/public-files/gfmy8FGusFRVOVU/critically_damped_bode_plot.png?scalingup=0&preview=1&a=1&c=94350942031314944%3Ae2c3e835&x=3840&y=3840" alt="critically_damped_bode_plot">
</div>

In conclusion, we have analyzed three different aspects compare the models obtained:

* Time response: similar evolution.
* Magnitude: similar value at low frecuencies.
* Phase: SIM different from IIM and BTM at high frecuencies.

#### Underdamped
An underdamped system has complex poles, and as a result it has oscilations. For the example, the following parameters \f$ m = 1, d = 1.5, k = 2 \f$ have been chosen:

$$ G(s) = \frac{1}{s^2 + 1.5s + 2} $$

<div style="margin: 0 auto; width: fit-content; max-width: 500px;">
    <img src="https://cernbox.cern.ch/remote.php/dav/public-files/B53VmRU3L6bHfQm/underdamped_s_plane.png?scalingup=0&preview=1&a=1&c=94350938273218560%3Aef02ad21&x=3840&y=3840" alt="underdamped_s_plane">
</div>

The transfer functions \f$ G(z) \f$ for each method are:

$$ G_{BTM}(z) = 10^{-5} \frac{2.481 z^2 + 4.963 z + 2.481}{z^2 - 1.985z + 0.9851} $$
$$ G_{IIM}(z) = 10^{-5} \frac{5.663z}{z^2 - 1.985z + 0.9851} $$
$$ G_{SIM}(z) = 10^{-5} \frac{4.975 z + 4.95}{z^2 - 1.985z + 0.9851} $$

As for the overdamped system, it can be observed that all of them have the same poles but different zeros, both in magnitude and number. This can be seen clearly in the \f$ z \f$ planes.

<div style="margin: 0 auto; width: fit-content; max-width: 800px;">
    <img src="https://cernbox.cern.ch/remote.php/dav/public-files/dGQlskNvGbzxCYq/underdamped_z_plane.png?scalingup=0&preview=1&a=1&c=94350938810089472%3A3fc05cac&x=3840&y=3840" alt="underdamped_z_plane">
</div>

Analyzing the time response, now we can observe some oscilations due to the imaginary part of the poles. The same evolution is  observed for the three methods; however, the magnitude reached is different for IIM.

<div style="margin: 0 auto; width: fit-content; max-width: 800px;">
    <img src="https://cernbox.cern.ch/remote.php/dav/public-files/LzJzrQzt0y0GNIl/underdamped_response_impulse_ramp.png?scalingup=0&preview=1&a=1&c=94350938541654016%3A455aada9&x=3840&y=3840" alt="underdamped_response_impulse_ramp">
</div>

<div style="margin: 0 auto; width: fit-content; max-width: 800px;">
    <img src="https://cernbox.cern.ch/remote.php/dav/public-files/ppmFPp58O4oOKHi/underdamped_response_sine_step.png?scalingup=0&preview=1&a=1&c=94350939346960384%3A2b477d82&x=3840&y=3840" alt="underdamped_response_sine_step">
</div>

If we now analyse the frequency response of the system, we can see that since the beginning the magnitude for IIM is different (as we could see in the time response graphs), and they differ even more after the cut-off frequency. As for the phase, it is very similar for the three methods, except for SIM, where there is a decrease in the last part.

<div style="margin: 0 auto; width: fit-content;">
    <img src="https://cernbox.cern.ch/remote.php/dav/public-files/zSLGRE3hHgk800o/underdamped_bode_plot.png?scalingup=0&preview=1&a=1&c=94350939078524928%3A75daa0b6&x=3840&y=3840" alt="underdamped_bode_plot">
</div>

In conclusion, we have analyzed three different aspects compare the models obtained:

* Time response: similar evolution.
* Magnitude: different values for IIM at low frecuencies. Different values for each method at high frecuencies.
* Phase: SIM different from IIM and BTM at high frecuencies.

#### Boundary of stability
This case is not realistic, as we are leaving out the damping, is is interesting with regard to the analysis, checking that the filter can produced the expected response from the equations. The system will oscilate indefinitely, i.e., the poles are on the imaginary axis. For the example, the following parameters \f$ m = 1, d = 0, k = 1 \f$ have been chosen:

$$ G(s) = \frac{1}{s^2 + 1} $$

<div style="margin: 0 auto; width: fit-content; max-width: 500px;">
    <img src="https://cernbox.cern.ch/remote.php/dav/public-files/9qUxWRQ4pjS23zu/boundary_s_plane.png?scalingup=0&preview=1&a=1&c=94350942299750400%3Afb1662a6&x=3840&y=3840" alt="boundary_s_plane">
</div>

The transfer functions \f$ G(z) \f$ for each method are:

$$ G_{BTM}(z) = 10^{-5} \frac{2.5 z^2 + 5 z + 2.5}{(z - 1)^2} $$
$$ G_{IIM}(z) = 10^{-5} \frac{10 z}{(z - 1)^2} $$
$$ G_{SIM}(z) = 10^{-5} \frac{5 z + 5}{(z - 1)^2} $$

In this case the imaginary poles of \f$ s \f$ plane cause that the poles of \f$ z \f$ plane are on the unitary circle.
As for the other systems, the zeros are different, both in magnitude and number.

<div style="margin: 0 auto; width: fit-content; max-width: 800px;">
    <img src="https://cernbox.cern.ch/remote.php/dav/public-files/WdtBWW7VXfdYlsJ/boundary_z_plane.png?scalingup=0&preview=1&a=1&c=94350942568185856%3A04e71553&x=3840&y=3840" alt="boundary_z_plane">
</div>

Analyzing the time response, we can observe the expected oscilations due to the imaginary poles, around the target value. It is interesting that in this occasion the magnitudes coincide again for the three methods.

<div style="margin: 0 auto; width: fit-content; max-width: 800px;">
    <img src="https://cernbox.cern.ch/remote.php/dav/public-files/1vxxgVFY3Cupu8u/boundary_response_impulse_ramp.png?scalingup=0&preview=1&a=1&c=94350943105056768%3Aff950daf&x=3840&y=3840" alt="boundary_response_impulse_ramp">
</div>

<div style="margin: 0 auto; width: fit-content; max-width: 800px;">
    <img src="https://cernbox.cern.ch/remote.php/dav/public-files/QH4jUZnghTne2gw/boundary_response_sine_step.png?scalingup=0&preview=1&a=1&c=94350942836621312%3A6d14ded7&x=3840&y=3840" alt="boundary_response_sine_step">
</div>

If we now analyse the frequency response of the system, we can observe clearly a gap in the phase diagram of 180 degrees, due to the imaginary poles (90 degrees each one). As the poles have the same frecuency, there is no transicion in the phase change. Because of this abrupt change of the phase, a peak can be observed in the magnitude plot, which is known as resonance. What is curious about the phase evolution is the fact that the gap is the opposite in SIM (+180 degrees) than the other two methods.

<div style="margin: 0 auto; width: fit-content;">
    <img src="https://cernbox.cern.ch/remote.php/dav/public-files/pE8nz7kMIfdOWHr/boundary_bode_plot.png?scalingup=0&preview=1&a=1&c=94350943373492224%3A83cc3a13&x=3840&y=3840" alt="boundary_bode_plot">
</div>

In conclusion, we have analyzed three different aspects compare the models obtained:

* Time response: similar evolution.
* Magnitude: similar values at low frecuencies. We have a resonance at the cut-off frecuency.
* Phase: abrupt gap of 180 degrees at the cut-off frecuency.


#### Aliasing effects
In this section we will analyze the effects of aliasing when the sampling frecuency is lower than the frecuency of the system. For this purpose, the sampling period chosen is \f$ T = 1 \f$, whereas the parameters of the system are \f$ m = 0.2, d = 0.2, k = 0.5 \f$

$$ G(s) = \frac{1}{0.2s^2 + 0.2s + 0.5} $$

<div style="margin: 0 auto; width: fit-content; max-width: 500px;">
    <img src="https://cernbox.cern.ch/remote.php/dav/public-files/7sKKlwDAmorJs9f/aliasing_s_plane.png?scalingup=0&preview=1&a=1&c=94350943910363136%3Ad1d4eefb&x=3840&y=3840" alt="aliasing_s_plane">
</div>

The transfer functions \f$ G(z) \f$ for each method are:

$$ G_{BTM}(z) = 10^{-5} \frac{0.5882 z^2 + 1.176 z + 0.5882}{z^2 - 0.3529 z + 0.5294} $$
$$ G_{IIM}(z) = 10^{-5} \frac{0.605 z}{z^2 - 0.08581 z + 0.3679} $$
$$ G_{SIM}(z) = 10^{-5} \frac{0.3022 z + 0.2107}{z^2 - 0.08581 z + 0.3679} $$

This is the case where the most differences between the three methods can be observed, as they no longer coincide even at the poles, as can be seen in the z planes.

<div style="margin: 0 auto; width: fit-content; max-width: 800px;">
    <img src="https://cernbox.cern.ch/remote.php/dav/public-files/8XaombyqHH9vvBH/aliasing_z_plane.png?scalingup=0&preview=1&a=1&c=94350943641927680%3A1fcae938&x=3840&y=3840" alt="aliasing_z_plane">
</div>

Even so, when analysing the time response, it can be seen that for the impulse, step and ramp signals, it maintains a similar evolution. There is a difference in magnitude, which is not constant, as it varies considerably depending on the signal in question. It can also be seen how, in general, the IIM takes longer to react, and then reacts more abruptly, while in BTM and SIM this transition is a little smoother.

<div style="margin: 0 auto; width: fit-content; max-width: 800px;">
    <img src="https://cernbox.cern.ch/remote.php/dav/public-files/09hhBSRzIZ7Hv13/aliasing_response_impulse_ramp.png?scalingup=0&preview=1&a=1&c=94350944447234048%3A070690e8&x=3840&y=3840" alt="aliasing_response_impulse_ramp">
</div>

On the other hand, if we look at the response to a sinusoidal function at the input, we see at first glance a completely different response from the three methods, due to the aliasing that may occur.

<div style="margin: 0 auto; width: fit-content; max-width: 800px;">
    <img src="https://cernbox.cern.ch/remote.php/dav/public-files/j2qtvVnGy7uelfN/aliasing_response_sine_step.png?scalingup=0&preview=1&a=1&c=94350944178798592%3Accff6d0a&x=3840&y=3840" alt="aliasing_response_sine_step">
</div>

These differences can also bee seen in the frecuency response, where the magnitudes and phases are different from the beginning. In addition to that, the cut-off frecuency is quite low, so as long as we put some little oscilations, we have even more differences between them.

<div style="margin: 0 auto; width: fit-content;">
    <img src="https://cernbox.cern.ch/remote.php/dav/public-files/LCuTbto7nLH8qMk/aliasing_bode_plot.png?scalingup=0&preview=1&a=1&c=94350944715669504%3A1d622d1e&x=3840&y=3840" alt="aliasing_bode_plot">
</div>

In conclusion, we have analyzed three different aspects compare the models obtained:

* Time response: more abrupt reaction of IIM. Considerable differences when we have frecuencies in the input signal, as in a sine function.
* Magnitude: different values for all methods, frecuencies and input signals.
* Phase: SIM evolution for the three methods.

#### General observations
In general (except for the aliasing case) we can see that the zero coefficients of the transfer functions has a strong correlation. It seems as if sum all of them we get a similar value. For this reason, at low frecuencies the magnitude is the same.

More specifically, the coefficients seems to be distributed in the following way:
* IIM: all is concentrated in the value with one delay (\f$ z^{-1} \f$ ).
* SIM: half is concentrated in the value with one delay (\f$ z^{-1} \f$ ), and the other half is in the value with two delays (\f$ z^{-2} \f$ ).
* BTM: half is concentrated in the value with one delay (\f$ z^{-1} \f$ ), and the other half is divided between the current value (without delay) and the other one with two delays (\f$ z^{-2} \f$ ).

The following table resume these distribution:

<table>
    <tr>
        <th></th>
        <th>\f$ z^0 \f$ </th>
        <th>\f$ z^{-1} \f$ </th>
        <th>\f$ z^{-2} \f$ </th>
    </tr>
    <tr>
        <td>Impulse Invariance Method</td>
        <td>0</td>
        <td>100 %</td>
        <td>0</td>
    </tr>
    <tr>
        <td>Step Invariance Method</td>
        <td>0</td>
        <td>50 %</td>
        <td>50 %</td>
    </tr>
    <tr>
        <td>Bilinear Transform Method</td>
        <td>25 %</td>
        <td>50 %</td>
        <td>25 %</td>
    </tr>
</table>

### References
In order to follow the process of obtaining coefficients \f$ a \f$ and \f$ b \f$ from the transfer function \f$ G(s) \f$, check the following <a href="https://gitlab.cern.ch/mro/robotics/analytic-and-numeric-analysis/mass-spring-damper-system" target="_blank">repository</a>, where it was solved using Maple Soft software.
