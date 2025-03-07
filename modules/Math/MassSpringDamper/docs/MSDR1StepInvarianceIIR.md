@ingroup group_sim_mass_spring_damper_r1

In this section we will follow the Step Invariance Method (SIM). To make this transformation, we will use the following equivalence, as in the IIM:

$$ z = e^{sT} $$

$$ T: \text{sampling time} $$

The difference with the IIM method is that we add first a step function to the transfer function \f$ G(s) \f$:

$$ G_{step}(s) = \frac{1}{s} \cdot G(s) $$

In order to apply this transformation, we will expand \f$ G_{step}(s) \f$ in partial fractions:

$$ G_{step}(s) = \frac{A}{s-p_1} + \frac{B}{s-p_2} + \frac{C}{s-p_3} $$

Where the poles are:

$$ p_1 = 0 $$
$$ p_2 = \frac{-d + \sqrt{d^2 - 4km}}{2m}$$
$$ p_3 = -\frac{d + \sqrt{d^2 - 4km}}{2m}$$

To apply the Z-transform to each partial fraction, now we need to add the inverse of step function:

$$ G(z) = \frac{z}{z-1}\left(\frac{Az}{z-e^{p_1 T}} + \frac{Bz}{z-e^{p_2 T}} + \frac{Cz}{z-e^{p_3 T}}\right) $$

After several calculations, the coefficients are obtained by the following equations:

$$ a_0 = p_2p_3(p_2 - p_3) $$
$$ a_1 = p_2p_3(p_2 - p_3) \left(-e^{p_2T} - e^{p_3T}\right) $$

$$ a_2 = p_2p_3(p_2 - p_3) e^{p_2T}  e^{p_3T} $$

$$ b_0 = 0 $$
$$ b_1 = p_3e^{p_2T} - p_2e^{p_3T} + p_2 - p_3 $$
$$ b_2 = p_2e^{(p_2+p_3)T} - p_3e^{(p_2+p_3)T} - p_2e^{p_2T} + p_3e^{p_3T}$$

The coefficient \f$ a_0 \f$ must be 1 to establish a direct relation between input and ouput, so we use this value to normalize all the coefficients:

$$ a_{0, N} = \frac{a_0}{a_0} = 1 $$
$$
a_{1, N} = \frac{a_0}{a_0} = -e^{p_2T} - e^{p_3T}
$$
$$
a_{2, N} = \frac{a_1}{a_0} = e^{p_2T}  e^{p_3T}
$$
$$ b_{0, N} = \frac{b_0}{a_0} = 0 $$
$$
b_{1,N} = \frac{b_1}{a_0} = \frac{p_3e^{p_2T} - p_2e^{p_3T} + p_2 - p_3}{p_2p_3(p_2 - p_3)}
$$
$$
b_{2, N} = \frac{p_2e^{(p_2+p_3)T} - p_3e^{(p_2+p_3)T} - p_2e^{p_2T} + p_3e^{p_3T}}{p_2p_3(p_2 - p_3)}
$$

### Complex poles
As we did in IIM, for negative values of the discriminant we will use the Euler's formula and the inverse relationship to cosine and sine functions:

Therefore, first we need to express the poles as functions of \f$ \sigma \f$ and \f$ j\omega \f$:

$$ p_1 = 0 $$
$$ p_2 = \sigma + j\omega $$
$$ p_3 = \sigma - j\omega $$

To get these expressions, \f$ \sigma \f$ and \f$ j\omega \f$ must be:

$$ \sigma = \frac{-d}{2m}; \quad j\omega = \frac{\sqrt{d^2-4km}}{2m} $$

Substituting these variables in the previous equations, we have:

$$ a_0 = 1 $$
$$
a_1 = -e^{(\sigma + j\omega)T} - e^{(\sigma - j\omega)T} = -e^{\sigma T}\left( e^{j\omega T} - e^{-j\omega T} \right)
$$
$$
a_2 = e^{(\sigma + j\omega)T} \cdot e^{(\sigma - j\omega)T} = e^{2\sigma T}
$$
$$ b_0  = 0 $$
$$
b_1 = \frac{(\sigma-j\omega)e^{(\sigma+j\omega)T} - (\sigma+j\omega)e^{(\sigma-j\omega)T} + (\sigma+j\omega) - (\sigma-j\omega)}{(\sigma+j\omega)(\sigma-j\omega)((\sigma+j\omega)-(\sigma-j\omega))} = \\
\frac{1}{\sigma^2-\omega^2} \left(\frac{\sigma e^{\sigma T}}{2j\omega}\left(e^{j\omega T} - e^{-j\omega T} \right) - \frac{e^{\sigma T}}{2}\left(e^{j\omega T} + e^{-j\omega T} \right) + 1 \right)
$$
$$
b_2 = \frac{(\sigma+j\omega)e^{2\sigma T} - (\sigma-j\omega)e^{2\sigma T} - (\sigma+j\omega)e^{(\sigma + j\omega)T} + (\sigma-j\omega)e^{(\sigma - j\omega)T}}{(\sigma+j\omega)(\sigma-j\omega)((\sigma+j\omega)-(\sigma-j\omega))} = \\
\frac{e^{\sigma T}}{\sigma^2-\omega^2} \left(\frac{-\sigma}{2j\omega}\left(e^{j\omega T} - e^{-j\omega T} \right) - \frac{1}{2}\left(e^{j\omega T} + e^{-j\omega T} \right) + e^{\sigma T} \right)
$$

Finally, using the Euler's formula we can get the following equations:

$$ a_0 = 1 $$
$$
a_1 = -2e^{\sigma T} \cos(\omega T)
$$
$$
a_2 = e^{2\sigma T}
$$
$$ b_0  = 0 $$
$$
b_1 = \frac{1}{\sigma^2+\omega^2} \left(\frac{\sigma e^{\sigma T}}{\omega} \sin(\omega T) - e^{\sigma T}\cos(\omega T) + 1 \right)
$$
$$
b_2 = \frac{e^{\sigma T}}{\sigma^2+\omega^2} \left(\frac{-\sigma}{\omega}\sin(\omega T) - \cos(\omega T) + e^{\sigma T} \right)
$$

Now it is possible to work in C++ with these equations. As we can observe, now we have oscilations due to cosine and sine functions.

### Indeterminations
For this method, there are some indeterminations in the boundary of real and complex poles. This is because the discriminant \f$ d^2 - 4km = 0 \f$ , so we have the following equations for the poles:

$$ p_2 = p_3 = \frac{-d}{2m} $$

Therefore, some indeterminations show up for the coefficients \f$ b_1 \f$ and \f$ b_2 \f$ :

$$
b_1 = \frac{p_3e^{p_2T} - p_2e^{p_3T} + p_2 - p_3}{p_2p_3(p_2 - p_3)} \longrightarrow
\lim_{p_3 \rightarrow p_2} b_1 = \frac{0}{0}
$$
$$
b_2 = \frac{p_2e^{(p_2+p_3)T} - p_3e^{(p_2+p_3)T} - p_2e^{p_2T} + p_3e^{p_3T}}{p_2p_3(p_2 - p_3)} \longrightarrow
\lim_{p_3 \rightarrow p_2} b_2 = \frac{0}{0}
$$

In order to solve these indeterminations we will use L'Hôpital's rule.

$$
\lim_{p_3 \rightarrow p_2} b_1 = \lim_{p_3 \rightarrow p_2} \frac{f_{b_1}(p_3)}{g_{b_1}(p_3)} \Longrightarrow
\lim_{p_3 \rightarrow p_2} \frac{f_{b_1}'(p_3)}{g_{b_1}'(p_3)} =
\lim_{p_3 \rightarrow p_2} \frac{e^{p_2T} - p_2Te^{p_3T} - 1}{p_2^2 - 2p_2p_3} = \frac{e^{p_2T}(p_2T - 1) + 1}{p_2^2}
$$

\f[
\lim_{p_3 \rightarrow p_2} b_2 = \lim_{p_3 \rightarrow p_2} \frac{f_{b_2}(p_3)}{g_{b_2}(p_3)} \Longrightarrow
\lim_{p_3 \rightarrow p_2} \frac{f_{b_2}'(p_3)}{g_{b_2}'(p_3)} =
\lim_{p_3 \rightarrow p_2} \frac{p_2Te^{(p_2+p_3)T} - e^{(p_2+p_3)T} - p_3Te^{(p_2+p_3)T} + e^{p_3T} + p_3Te^{p_3T}}{p_2^2 - 2p_2p_3} = \\
\frac{e^{2p_2T} - e^{p_2T} - p_2Te^{p_2T}}{p_2^2} = \frac{e^{p_2T}(e^{p_2T} - p_2T - 1)}{p_2^2}
\f]

On the other hand, if we start from the equations for the complex poles, instead of the complex ones, we have:

$$
b_1 = \frac{1}{\sigma^2+\omega^2} \left(\frac{\sigma e^{\sigma T}}{\omega} \sin(\omega T) - e^{\sigma T}\cos(\omega T) + 1 \right) \longrightarrow
\lim_{\omega \rightarrow 0} b_1 = \frac{0}{0}
$$

$$
\lim_{\omega \rightarrow 0} b_1 = \lim_{\omega \rightarrow 0} \frac{f_{b_1}(\omega)}{g_{b_1}(\omega)} \Longrightarrow
\lim_{\omega \rightarrow 0} \frac{f_{b_1}'(\omega)}{g_{b_1}'(\omega)} =
\lim_{\omega \rightarrow 0} \frac{\sigma Te^{\sigma T}\cos(\omega T) - e^{\sigma T}\cos(\omega T) + \omega Te^{\sigma T}\sin(\omega T) + 1}{\sigma^2+3\omega^2} = \frac{e^{\sigma T}(\sigma T - 1) + 1}{\sigma^2}
$$

$$
b_2 = \frac{e^{\sigma T}}{\sigma^2+\omega^2} \left(\frac{-\sigma}{\omega}\sin(\omega T) - \cos(\omega T) + e^{\sigma T} \right) \longrightarrow
\lim_{\omega \rightarrow 0} b_2 = \frac{0}{0}
$$
$$
\lim_{\omega \rightarrow 0} b_2 = \lim_{\omega \rightarrow 0} \frac{f_{b_2}(\omega)}{g_{b_2}(\omega)} \Longrightarrow
\lim_{\omega \rightarrow 0} \frac{f_{b_2}'(\omega)}{g_{b_2}'(\omega)} =
\lim_{\omega \rightarrow 0} e^{\sigma T} \left(\frac{-\sigma T\cos(\omega T) - \cos(\omega T) + \omega T\sin(\omega T) + e^{\sigma T}}{\sigma^2+3\omega^2} \right) = \frac{e^{\sigma T}(e^{\sigma T} - \sigma T - 1)}{\sigma^2}
$$

Regarding \f$ \omega = 0 \rightarrow p_2 = \omega \f$ we obtain the same values for \f$ b_1 \f$ and \f$ b_2 \f$ whether they are calculated as real or complex poles.
