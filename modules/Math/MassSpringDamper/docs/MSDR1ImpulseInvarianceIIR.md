@ingroup group_iim_mass_spring_damper_r1

To make this transformation, we will use the following equivalence:

$$ z = e^{sT} $$

$$ T: \text{sampling time} $$

To apply this transformation, we need to expand first \f$ G(s) \f$ in partial fractions:

$$ G(s) = \frac{A}{s-p_1} + \frac{B}{s-p_2} $$

Where the poles are:

$$ p_1 = \frac{-d + \sqrt{d^2 - 4km}}{2m}$$
$$ p_2 = -\frac{d + \sqrt{d^2 - 4km}}{2m}$$


Now we can apply the Z-transform to each partial fraction:

$$
G(z) = T \cdot \left(A \cdot \frac{z}{z-e^{p_1 T}} + B \cdot \frac{z}{z-e^{p_2 T}} \right)
$$

After several calculations, the coefficients are obtained by the following equations:

$$ a_0 = p_1 - p_2 $$
$$ a_1 = (p_1-p_2) \left(-e^{p_1T} - e^{p_2T}\right) $$

$$ a_2 = (p_1-p_2) e^{p_1T}  e^{p_2T} $$

$$ b_0 = 0 $$
$$ b_1 = T\cdot \left( e^{p_1T} -  e^{p_2T} \right) $$
$$ b_2 = 0 $$

The coefficient \f$ b_0 \f$ must be 1 to establish a direct relation between input and ouput, so we use this value to normalize all the coefficients:

$$ a_{0,N} = \frac{a_0}{a_0} = 1 $$
$$ a_{1, N} = \frac{a_0}{a_0} = -e^{p_1T} - e^{p_2T} $$
$$ a_{2, N} = \frac{a_1}{a_0} = e^{p_1T}  e^{p_2T} $$
$$ b_{0,N} = \frac{b_0}{a_0} = 0 $$
$$
b_{1, N} = \frac{b_1}{a_0} = \frac{T\cdot \left( e^{p_1T} -  e^{p_2T} \right)}{p_1-p_2}
$$
$$ b_{2, N} = \frac{b_2}{a_0} = 0 $$

### Complex poles
As we can see in the poles, there is a root that can be a potential imaginary value if the discriminant \f$ d^2-4km < 0 \f$. Therefore, when the discriminant is positive (or zero) we can work with these equations without any problem, but if it is negative, we need to do some transformations.

For this purpose, we will use the Euler's formula and the inverse relationship to cosine and sine functions:

$$ e^{j\omega T} = \cos(\omega T) + j\sin(\omega T)$$
$$ \cos(\omega T) = \frac{1}{2}\left[ e^{j\omega T} + e^{-j\omega T}\right]$$
$$ \sin(\omega T) = \frac{1}{2j}\left[ e^{j\omega T} - e^{-j\omega T}\right]$$

Therefore, first we need to express the poles as functions of \f$ \sigma \f$ and \f$ j\omega \f$:

$$ p_1 = \sigma + j\omega $$
$$ p_2 = \sigma - j\omega $$

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
$$ b_0 = 0 $$
$$
b_1 = \frac{Tm}{j\omega \cdot 2m} \cdot \left(e^{(\sigma + j\omega)T} - e^{(\sigma - j\omega)T}\right) = \frac{Te^{\sigma T}}{\omega} \cdot \frac{1}{2j} \left( e^{j\omega T} - e^{-j\omega T} \right)
$$
$$ b_2  = 0 $$

Finally, using the Euler's formula we can get the following equations:

$$ a_0 = 1 $$
$$
a_1 = -2e^{\sigma T} \cos(\omega T)
$$
$$
a_2 = e^{2\sigma T}
$$
$$ b_0 = 0 $$
$$
b_1 = \frac{Te^{\sigma T}}{\omega} \sin(\omega T)
$$
$$ b_2  = 0 $$

Now it is possible to work in C++ with these equations. As we can observe, now we have oscilations due to cosine and sine functions.

### Indeterminations
For this method, there is one indetermination in the boundary of real and complex poles. This is because the discriminant \f$ d^2 - 4km = 0 \f$ , so we have the following equations for the poles:

$$ p_1 = p_2 = \frac{-d}{2m} $$

Therefore, an indetermination show up for the coefficient \f$ b_1 \f$ :

$$
b_1 = \frac{T \cdot (e^{p_2T} - e^{p_1T}) }{p_2 - p_1} \longrightarrow
\lim_{p_2 \rightarrow p_1} b_1 = \frac{0}{0}
$$

In order to solve this indetermination we will use L'Hôpital's rule.

$$
\lim_{p_2 \rightarrow p_1} b_1 = \lim_{p_2 \rightarrow p_1} \frac{f_{b_1}(p_2)}{g_{b_1}(p_2)} \Longrightarrow
\lim_{p_2 \rightarrow p_1} \frac{f_{b_0}'(p_2)}{g_{b_0}'(p_2)} =
\lim_{p_2 \rightarrow p_1} \frac{T^2e^{p_2T}}{1} = T^2e^{p_1T}
$$

On the other hand, if we start from the equations for the complex poles, instead of the real ones, we have:


$$
b_1 = \frac{Te^{\sigma T}}{\omega} \sin(\omega T) \longrightarrow
\lim_{\omega \rightarrow 0} b_1 = \frac{0}{0}
$$
$$
\lim_{\omega \rightarrow 0} b_1 = \lim_{\omega \rightarrow 0} \frac{f_{b_1}(\omega)}{g_{b_1}(\omega)} \Longrightarrow
\lim_{\omega \rightarrow 0} \frac{f_{b_1}'(\omega)}{g_{b_1}'(\omega)} = \lim_{\omega \rightarrow 0} \frac{T^2e^{\sigma T}\cos(\omega T)}{1} = T^2e^{\sigma T}
$$

Regarding \f$ \omega = 0 \rightarrow p_1 = \omega \f$ we obtain the same values for \f$ b_1 \f$ whether they are calculated as real or complex poles.
