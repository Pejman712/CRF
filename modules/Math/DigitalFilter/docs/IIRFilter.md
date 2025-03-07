@ingroup group_iir_filter

Implementation of the Infinite Impulse Response Filter (IIR), which is a recursive filter in that the output from the filter is computed by using the current and previous inputs and previous outputs.

Because the filter uses previous values of the output, there is feedback of the output in the filter structure. The design of the IIR filter is based on identifying the pulse transfer function G(z) that satisfies the requirements of the filter specification.

There are four direct form implementations of this filter. Currently it is developed the Direct Form II.

### Direct Form I
This method utilize the difference equation directly:

$$ \sum_{i=0}^N y(n-i) = \sum_{i=0}^M x(n-i) $$ 

A signal graph for a **second order** signal is shown in the following figure:

<a href="https://www.dsprelated.com/freebooks/filters/Direct_Form_I.html" target="_blank" style="display:block; margin: 0 auto; width: fit-content;">
<img src="https://www.dsprelated.com/josimages_new/filters/img1124.png" alt="Signal_Graph">
</a>

Considering \f$ a_0 = 1 \f$ by convention, the equations for this filter are:

$$  y(n) = b_0 \cdot x(n) + b_1 \cdot x(n-1) + b_2 \cdot x(n-2) - a_1 \cdot y(n-1) - a_2 \cdot y(n-2) $$

If the filter has an order \f$ n\f$ , then:

$$ y(n) = \sum_{i=0}^{n} b_i \cdot x(n-i) - \sum_{j=1}^{n} a_j \cdot y(n-j) $$

### Direct Form II
This method utilize an intermediate signal, in order to reduce the number of delays used. A signal graph for a **second order** signal is shown in the following figure:

<a href="https://www.dsprelated.com/freebooks/filters/Direct_Form_II.html" target="_blank" style="display:block; margin: 0 auto; width: fit-content;">
<img src="https://www.dsprelated.com/josimages_new/filters/img1141.png" alt="Signal_Graph">
</a>

Considering \f$ a_0 = 1 \f$ by convention, the equations for this filter are:

$$  v(n) = x(n) - a_1 \cdot v(n-1) - a_2 \cdot v(n-2) $$
$$  y = b_0 \cdot v(n) + b_1 \cdot v(n-1) + b_2 \cdot v(n-2) $$

If the filter has an order \f$ n\f$ , then:

$$ v(n) = x(n) - \sum_{i=1}^{n} a_i \cdot v(n-i) $$
$$ y(n) = \sum_{i=0}^{n} b_i \cdot v(n-i) $$

In order to use this filter, we are provided with `direct2FormFilter()` function.

#### Initialization
When the filter is launched for the first time, the intermediate signal \f$ v \f$ has not been initialized yet, so it is necessary to establish a condition to set values for \f$ v \f$ , guaranteeing the stability of the filter at the same time.

$$ \text{condition} \Longrightarrow v = v(n) = v(n-1) = v(n-2) = ... $$

Therefore, after applying this condition, the filter is initialized with the following equations:

$$ v = \frac{x(n)}{\sum_{i=0}^{n} a_i} $$
$$ y = \sum_{i=0}^{n} b_i \cdot v $$

### References
For more information, consult the book <a href="https://www.dsprelated.com/freebooks/filters/Four_Direct_Forms.html" target="_blank">Introduction to Digital Filters</a>
