@ingroup group_btm_mass_spring_damper_r1

In this section we will follow the Bilinear Transform Method (BTM). To make this transformation, we will use the following equivalence:

$$ s = \frac{1+sT/2}{1-sT/2} \Longrightarrow z = \frac{2}{T}\frac{1-z^{-1}}{1+z^{-1}} $$

In this case, it is possible to apply directly the Z-transform to \f$ G(s) \f$:

$$ G(z) = \frac{T^2(1+z^{-1})^2}{kT^2(1+z^{-1})^2+2dT(1-z^{-2})+4m(1-z^{-1})^2} $$

After grouping and simplyfing the previous equation, we have the following coefficients:

$$ a_0 = 1 $$
$$ a_1 = \frac{2kT^2-8m}{kT^2+2dT+4m} $$
$$ a_2 = \frac{kT^2-2dT+4m}{kT^2+2dT+4m} $$
$$ b_0 = \frac{T^2}{kT^2+2dT+4m} $$
$$ b_1 = \frac{2T^2}{kT^2+2dT+4m} $$
$$ b_2  = \frac{T^2}{kT^2+2dT+4m} $$


These coefficients are passed to the function direct2FormFilter() to obtain the new ouput and the intermediate signal \f$ v \f$.

For sinusoidal functions, prewarping will be needed, due to the non-linear relationship between analog and digital frecuencies:

$$ \Omega = \frac{2}{T}\tan\left(\frac{\omega}{2}\right) $$

For simplicity, we will apply the prewarping to the input signal, instead of the filter. In order to apply this process, we will follow the following steps:

1. Choose the target analog frecuency for the function: \f$ \sin(\Omega) \f$
2. Calculate the digital frecuency using the inverse relationshsip: 
   $$ \omega = 2\arctan\left(\frac{\Omega T}{2}\right) $$
3. Apply the new input signal to the filter: \f$ \sin(\omega T) \f$
