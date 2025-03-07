@ingroup group_cubic_orientation_spline

This class generates a time-dependent interpolation that crosses a set of Orientations at specified time instances, and begins and ends the interpolation with specified angular velocities.

To achieve this, the following variables must be passed through the constructor.  
<br>
<br>

**Inputs with their shorthand notation:**
* orientationPoints       -    A set of N input Orientations that the interpolation should cross through (\f$\vec{Q}\f$)
* initialAngularVelocity  -    The angular velocity at the first input instance (\f$\vec{\omega}_1\f$ or \f$\vec{\omega}_{initial}\f$)
* finalAngularVelocity    -    The angular velocity at the last input instance (\f$\vec{\omega}_N\f$ or \f$\vec{\omega}_{final}\f$)
* timeInstances           -    A set of N input time values representing the time at which the interpolation crosses each input Orientation point (\f$\vec{T}\f$)
* convergenceTol          -    Convergence tolerance when solving for the intermediate angular rates solution (\f$\epsilon_c\f$). This value impacts the accuracy of the interpolation and the speed of the algorithm
* maxIter                 -    Maximum number of iterations when solving for the intermediate angular rates solution (\f$m\f$). This value can be used and tuned to reject interpolations with oscillations

Once the CubicOrientationSpline object has been constructed, the interpolation is defined and the following can be found:
* \f$q(t)\f$ - The Orientation at a specific time. Found using evaluateOrientation(t)
* \f$\vec{\omega}(t)\f$ - The angular velocity at a specific time. Found using evaluate(t, 1)
* \f$\vec{\alpha}(t)\f$ - The angular acceleration at a specific time. Found using evaluate(t, 2)  
<br>

**Notes:**
* If the interpolation is evaluated outside of the defined range of time values, it will evaluate the interpolation at the nearest defined time value.
* If the initialAngularVelocity or the finalAngularVelocity is high when the intervals between the times in timeInstances are large, the computed interpolation may contain oscillations with high intermediate angular velocities, and in the worst case, may result in no interpolation being found.
* If the intermediate rates solution is not converging logarithmically, it tends to have oscillations in the resultant interpolation. The less the the rates solution converges logarithmically, the more oscillations it induces until no solution is found.  
<br>

<table>
<caption id="multi_row">Summary of contributors</caption>
<tr>    <th>@Name      <th>Role     <th>Company             <th>Year
<tr><td>@Chelsea Davidson     <td>Author  <td>CERN - BE/CEM/MRO       <td>2024
<tr><td>James McEnnan     <td>Author of  paper, Quaternion Cubic Spline  <td>        -       <td>2003
</table>  

The algorithm used in this class was based on James McEnnan's paper, *Quaternion Cubic Spline*, written May 28, 2003. This documentation outlines the key relationships and equations used in this algorithm but you may refer to the original paper for further understanding. McEnnan's paper and corresponding code can be found here: https://qspline.sourceforge.net/  
<br>

### Algorithm Explanation
*Note that the equations used in this explanation use quaternions to represent orientations but due to the implementation of the Rotation class in the CRF (alias Orientation), these quaternion equations can be done using Orientation objects.*

Consider the final interpolation for an example with 5 input Orientation points (as seen below where the Orientation points are depicted as Quaternions). Between each of the input Orientation points, there is a function, \f$q_i(t)\f$, that defines the interpolation over that interval.

<div style="text-align: center;">
<img src="https://cernbox.cern.ch/remote.php/dav/public-files/eNn6mNCY8ruJzpI/CubicOrientationSplineExample.png?scalingup=0&preview=1&a=1&c=136054739736461312%3Abec6c895&x=3840&y=3840" alt="Image" style="width:60%;height:auto;" />
</div>

So, for each interpolation interval, the Orientation is defined as:

\f[
    q(t) = q_{start}\Delta q(t) \tag*{1}
\f]

where \f$q_{start}\f$ is the start Orientation for that segment and \f$\Delta q(t)\f$ is the transformation from \f$q_{start}\f$ to \f$q\f$, given below as a quaternion in \f$[w,x,y,z]\f$ form:  

\f[
    {Δq(t)} = \begin{bmatrix} cos(\theta(t)/2) \\ u_1(t)sin(\theta(t)/2) \\ u_2(t)sin(\theta(t)/2) \\ u_3(t)sin(\theta(t)/2) \end{bmatrix} \tag*{2}
\f]

where:
\f[
    \theta(t) = |\vec{\theta}(t)|
\f]
\f[
    \hat{u}(t) =  \vec{\theta}(t)/|\vec{\theta}(t)| = \vec{\theta}(t)/\theta(t)
\f]

Thus, the angular velocity (\f$ \vec{\omega} \f$) and angular acceleration (\f$ \vec{\alpha} \f$) for each segment is given as:
\f[
    \vec{\omega}(t) = \hat{u}\dot{\theta} + sin\theta(\vec{v}\times\hat{u})-(1-cos\theta)\vec{v} \tag*{3}
\f]
\f[
    \vec{\alpha}(t) = \hat{u}\ddot{\theta}+sin\theta(\dot{\vec{v}}\times\hat{u})-(1-cos\theta)\dot{\vec{v}}+\dot{\theta}(\vec{v}\times\hat{u})+\vec{\omega}\times(\hat{u}\dot{\theta}-\vec{v}) \tag*{4}
\f]

where:
\f[
    \dot{\theta}=\vec{\theta}\cdot\dot{\vec{\theta}}/\theta = \hat{u}\cdot\dot{\vec{\theta}} \tag*{5}
\f]
\f[
    \ddot{\theta} = (\vec{v}\times\hat{u})\cdot\dot{\vec{\theta}}+\hat{u}\cdot\ddot{\vec{\theta}} \tag*{6}
\f]
\f[
    \vec{v} = (\vec{\theta}\times\dot{\vec{\theta}})/\theta^2 = (\hat{u}\times\dot{\vec{\theta}})/\theta \tag*{7}
\f]
\f[
    \dot{\vec{v}} = (\vec{\theta}\times\ddot{\vec{\theta}} - 2\vec{\theta}\cdot\dot{\vec{\theta}}\vec{v})/\theta^2 \tag*{8}
\f]

*Note that the above variables are all time dependent, written this way for readability.*

As seen above, the \f$\vec{\omega}\f$ and \f$\vec{\alpha}\f$ equations have singularities when \f$θ = 0\f$. These can be avoided using the following:
\f[
    \tag*{9}
\f]
* \f$\vec{\omega}\f$ :      when \f$\theta = 0\f$     →   set  \f$\vec{\omega} = \dot{\vec{\theta}}\f$
\f[
    \tag*{10}
\f]
* \f$\vec{\alpha}\f$ :      when \f$\theta = 0\f$     →   set  \f$\vec{\alpha} = \ddot{\vec{\theta}}\f$

These variables all depend on the rotation vector, \f$\vec{\theta}(t)\f$. For a cubic spline interpolation, this rotation vector is assumed to be a vector-valued third-order polynomial function of time:
\f[
    \vec{\theta}(t) = \vec{a}^3_1 (x-1)^3 + \vec{a}^3_2 x(x-1)^2 + \vec{a}^3_3 x^2(x-1)+\vec{a}^3_4 x^3 \tag*{11}
\f]
\f[
    \dot{\vec{\theta}}(t) = \vec{a}^2_1 (x-1)^2 + \vec{a}^2_2 x(x-1) + \vec{a}^2_3 x^2 \tag*{12}
\f]
\f[
    \ddot{\vec{\theta}}(t) = \vec{a}^1_1 (x-1) + \vec{a}^1_2 x \tag*{13}
\f]

where \f[
    x = \frac{(t - t_{\text{start}})}{(t_{\text{end}} - t_{\text{start}})} = \frac{(t - t_{\text{start}})}{\Delta t} \tag*{14}
\f]

So, interpolating between all input Orientations requires \f$N-1\f$ functions of \f$\vec{\theta}(t)\f$ defining each interpolation segment. Thus, the general equation for finding the Orientation at any given time in the interpolation, \f$T_i \leq t \leq T_{i+1}\f$ where \f$1 \leq i \leq N-1\f$  is:

\f[
    q(t) = Q_i \Delta q_i(t) \tag*{15}
\f]

where \f$Q_i\f$ is the input Orientation point that begins the interpolation section and \f$\Delta q_i(t)\f$ is the transformation from \f$Q_i\f$ to \f$q\f$. As seen in Equation 2, this \f$\Delta q_i(t)\f$ is defined by the cubic polynomial function of \f$\vec{\theta}_i(t)\f$ over that interval.

So, to determine the interpolated Orientations, angular velocities, and angular accelerations, the coefficients of \f$\vec{\theta}(t)\f$ and its derivatives for each segment must first be found. This can be done using the following equations:
\f$\tag*{16}\f$
* \f$\vec{a}_1^3 = 0\f$
* \f$\vec{a}_2^3 = \Delta t \vec{\omega}_{start}\f$
* \f$\vec{a}_3^3 = \Delta t B^{-1}\cdot\vec{\omega}_{end} - 3\hat{e}\Delta\theta\f$
* \f$\vec{a}_4^3 = \hat{e}\Delta\theta\f$  
<br>

* \f$\vec{a}_1^2 = \vec{\omega}_{start}\f$
* \f$\vec{a}_2^2 = 2\vec{\omega}_{start}+2B^{-1}\cdot\vec{\omega}_{end}-6\hat{e}\Delta\theta/\Delta t\f$
* \f$\vec{a}_3^2 = B^{-1}\cdot\vec{\omega}_{end}\f$  
<br>

* \f$\vec{a}_1^1 = (4\vec{\omega}_{start} + 2B^{-1}\cdot\vec{\omega}_{end} - 6\hat{e}\Delta\theta/\Delta t)/\Delta t \f$
* \f$\vec{a}_2^1 = (2\vec{\omega}_{start} + 4B^{-1}\cdot\vec{\omega}_{end} - 6\hat{e}\Delta\theta/\Delta t)/\Delta t \f$  
<br>

As seen above, these coefficients depend on the initial and final angular velocity of these interpolated segments, \f$\vec{\omega}_{start}\f$ and \f$\vec{\omega}_{end}\f$ which correspond to \f$\vec{\omega}_{i}\f$ and \f$\vec{\omega}_{i+1}\f$. The intermediate rates (angular velocities at all but the endpoint Orientations) are not known and therefore must be solved for. This is done using an iterative algorithm that uses the given conditions, \f$\vec{\omega}_1\f$ and \f$\vec{\omega}_N\f$, and imposes continuity constraints at input instances. The details on how these rates are solved for is explored in the section 'Finding the angular rates'  below.

These coefficients also depend on \f$\Delta\theta\f$ and \f$\hat{e}\f$ which is the rotation angle and eigen-axis determined from the initial and final Orientations for that segment such that:

\f[
    \Delta q(t_{end}) = q_{start}^{-1}q_{end} = \begin{bmatrix} cos(\Delta \theta/2) \\ e_1sin(\Delta \theta/2)\\ e_2sin(\Delta \theta/2) \\ e_3sin(\Delta \theta/2) \end{bmatrix} \tag*{17}
\f]

where \f$q_{start}\f$ is \f$Q_i\f$, \f$q_{end}\f$ is \f$Q_{i+1}\f$, and \f$t_{end}\f$ is \f$T_{i+1}\f$.

This can be thought of as a unit vector and an angle of rotation about that vector that rotates the start Orientation to match the end Orientation. Although Equation 17 is depicted as a quaternion in \f$[w,x,y,z]\f$ form, \f$\Delta\theta\f$ and \f$\hat{e}\f$ can be found directly by extracting the angle and axis from the Angle Axis representation of the Relative Rotation, \f$\Delta q(t_{end})\f$.

The coefficient equations also refer to a function \f$B^{-1}\cdot\vec{\omega}_{end}\f$ which is the equation to transform the angular rate vector to the coefficient vector. The equation is defined for an arbitrary vector \f$\vec{x}\f$:

\f[
    B^{-1}(\hat{e}, \Delta\theta)\cdot\vec{x} = \hat{e}\cdot\vec{x}\hat{e} + \frac{\Delta\theta sin\Delta\theta}{2(1-cos\Delta\theta)}(\hat{e} \times \vec{x}) \times \hat{e} + \frac{1}{2}\Delta\theta\hat{e}\times\vec{x}\tag*{18}
\f]

If \f$\Delta\theta=0\f$, \f$B\f$ reduces to the identity matrix and thus \f$B^{-1}(\hat{e}, 0)\cdot\vec{x} = \vec{x}\f$.

Since these coefficients are describing the polynomial equation for \f$\vec{\theta}(t)\f$ over a specific interpolation interval, the \f$\Delta\theta\f$ and \f$\hat{e}\f$ in Equation 18 above refer to the rotation angle and eigen-axis defined for that interval (see Equation 17).  
<br>

### Summary of procedure:
Thus, to find the interpolated Orientations, angular rates, and angular accelerations, the equations of \f$\vec{\theta}(t)\f$ and its derivatives over each interval must first be found (Equation 11-13). These polynomial equations are characterised by their coefficients which require \f$\Delta\theta\f$, \f$\hat{e}\f$, a function \f$B^{-1}\cdot\vec{\omega}_{end}\f$, and the initial and final angular velocity for that segment, \f$\vec{\omega}_{start}, \vec{\omega}_{end}\f$. Once these variables have been found for each interval, the equations of \f$\vec{\theta}(t)\f$ and its derivatives can be determined and used to solve for the interpolated Orientations, angular rates, and angular accelerations.  
<br>

**Steps to construct interpolation (done in the `CubicOrientationSpline()` constructor):**
* Find \f$\Delta\theta\f$, \f$\hat{e}\f$ for each interval (Equation 17)
* Setup equations of \f$\theta(t)\f$ and its derivatives for each interpolation segment by finding the coefficients - call `setPolynomCoeffs()`
    * Determine the angular velocity at each input Orientation point using the iterative algorithm described in the "Finding the angular rates" section below - call `getRates()`
    * Calculate all coefficients of \f$\vec{\theta}(t)\f$ and its derivatives for each interval (Equation 16)
        * Use \f$B^{-1}\cdot\vec{\omega}_{end}\f$ (Equation 18) - call `bInvDotX()`  
<br>

**Steps to calculate the interpolated Orientation, angular velocity, and angular acceleration at a given time instance (done in `evaluateOrientation()` and `evaluate()`):**
* Determine which interpolation segment this time instance belongs
* Obtain the relevant coefficients that characterise \f$\vec{\theta}(t)\f$ and its derivatives for this segment.
* Calculate value of \f$\vec{\theta}(t)\f$ and its derivatives at the given time instance using the found coefficients for this interpolation segment (Equation 11-13)
* Calculate the Orientation, angular velocity, or angular acceleration at this time instance (Equations 1-10)  
<br>
<br>


### Finding the angular rates:
The following algorithm aims to find the angular velocity (rate) at each input instance such that continuity and the given conditions (the angular velocity at the endpoints) are satisfied. The algorithm arises from enforcing continuity in the Orientation, angular rate and angular acceleration functions. That is:

\f[
    q_{k-1}(T_k) = q_k(T_k) = Q_k \tag*{19}
\f]
\f[
    \vec{\omega}_{k-1}(T_k) = \vec{\omega}_k(T_k)
\f]
\f[
    \vec{\alpha}_{k-1}(T_k) = \vec{\alpha}_k(T_k)
\f]

where \f$2 \leq k \leq N-1\f$ which represents the intermediate input instances. So \f$q_{k-1}, \omega_{k-1}, \alpha_{k-1}\f$ are the interpolating functions for the section before intermediate instance \f$k\f$, and \f$q_k, \omega_k, \alpha_k\f$ are the interpolating functions for the section after intermediate instance \f$k\f$, and \f$T_k\f$ is the input time value at this instance \f$k\f$.

Carrying this constraint through, the following equation for the angular rate at each intermediate instance, \f$k\f$ is obtained:

\f[
    \vec{\omega}_k=\frac{6\hat{e}_k\Delta\theta_k}{(\Delta t_k)^2} + \frac{6\hat{e}_{k+1}\Delta\theta_{k+1}}{(\Delta t_{k+1})^2} - \vec{R}_k \tag*{20}
\f]

where \f$R_k\f$ is the non-linear component that depends on the intermediate angular rate itself:
\f[
    \vec{R}_k = \vec{R}(\hat{e}_k, \Delta \theta_k, \vec{\omega}_k) \tag*{21}
\f]
\f[
    \vec{R}(\hat{e}_k, \Delta \theta_k, \vec{\omega}_{k}) = r_0[\omega^2_{k} - (\hat{e}_k\cdot\vec{\omega}_{k})^2]\hat{e}_k + r_1\hat{e}_k\cdot\vec{\omega}_{k}(\hat{e}_k\times\vec{\omega}_{k})\times\hat{e}_k+r_2\hat{e}_k\cdot\vec{\omega}_{k}\hat{e}_k\times\vec{\omega}_{k}
\f]

where
\f[
    r_0 = \frac{\Delta\theta_k - sin\Delta\theta_k}{2(1-cos\Delta\theta_k)}
\f]
\f[
    r_1 = \frac{\Delta\theta_k sin\Delta\theta_k - 2(1-cos\Delta\theta_k)}{\Delta\theta_k(1-cos\Delta\theta_k)}
\f]
\f[
    r_2 = 0
\f]

which has a singularity when \f$\Delta\theta_k = 0\f$. So when \f$\Delta\theta_k = 0\f$, set  \f$\vec{R}_k = \vec{0}\f$.

Thus, to solve for the intermediate angular rates, an iterative approach can be used where the previous iteration's value of the rates is used to evaluate \f$\vec{R}_k\f$ and then \f$\vec{R}_k\f$ can be treated as a known term. In this method, the rate values are initially set to be zero so that \f$\vec{R}_k = \vec{0}\f$. The algorithm then iterates until the difference between the previous and current intermediate angular rate vector solution is less than a prescribed tolerance, \f$\epsilon_c\f$.

Thus, at each iteration, the intermediate angular rate vector equation has the following matrix form where the rows correspond to the intermediate instances, \f$k\f$:
\f[
    \tag*{22}
\f]
<div style="text-align: center;">
<img src="https://cernbox.cern.ch/remote.php/dav/public-files/Mee5lx7n2ThlE6c/RatesMatrix.png?scalingup=0&preview=1&a=1&c=136054739468025856%3A38be6c32&x=3840&y=3840" alt="Image" style="width:70%;height:auto;"/>
</div>

where these variables are:
\f[
    a_k = \frac{2}{\Delta t_k} \tag*{23}
\f]
\f[
    b_k = \frac{4}{\Delta t_k} + \frac{4}{\Delta t_{k+1}} \tag*{24}
\f]
\f[
    c_k = \frac{2}{\Delta t_{k+1}} \tag*{25}
\f]
\f[
    \vec{d}_2 = \frac{6\hat{e}_2\Delta\theta_2}{(\Delta t_2)^2} + \frac{6 \hat{e}_3\Delta\theta_3}{(\Delta t_3)^2} - \vec{R}(\hat{e}_2, \Delta\theta_2, \vec{\omega}^{prev}_2) - a_2B_2\cdot\vec{\omega}_{initial} \tag*{26}
\f]
\f[
    \vec{d}_k = \frac{6\hat{e}_k\Delta\theta_k}{(\Delta t_k)^2} + \frac{6 \hat{e}_{k+1}\Delta\theta_{k+1}}{(\Delta t_{k+1})^2} - \vec{R}(\hat{e}_k, \Delta\theta_k, \vec{\omega}^{prev}_k) \text{,     for } 3 \leq k \leq N-2
\f]
\f[
    \vec{d}_{N-1} = \frac{6\hat{e}_{N-1}\Delta\theta_{N-1}}{(\Delta t_{N-1})^2} + \frac{6 \hat{e}_N\Delta\theta_N}{(\Delta t_N)^2} - \vec{R}(\hat{e}_{N-1}, \Delta\theta_{N-1}, \vec{\omega}^{prev}_{N-1}) - c_{N-1}B^{-1}_N\cdot\vec{\omega}_{final}
\f]

This requires the function \f$B_2\cdot\vec{\omega}_{initial}\f$ which is the function to transform the coefficient vector to the angular rate vector. The equation is defined for an arbitrary vector \f$\vec{x}\f$:

\f[
    B_k = B(\hat{e}_k, \Delta\theta_k) \tag*{27}
\f]
\f[
    B(\hat{e}, \Delta\theta)\cdot\vec{x} = \vec{x}\cdot\hat{e}\hat{e}+\frac{sin\Delta\theta}{\Delta\theta}(\hat{e}\times\vec{x})\times\hat{e} - \frac{1-cos\Delta\theta}{\Delta\theta}\hat{e}\times\vec{x}
\f]

If \f$\Delta\theta=0\f$, \f$B\f$ reduces to the identity matrix and thus \f$B(\hat{e}, 0)\cdot\vec{x} = \vec{x}\f$.

The tridiagonal matrix in Equation 22 can be reduced to the upper triangular form by eliminating the elements in the lower diagonal (those in blue). The elements comprising the lower diagonal can be eliminated by multiplying the preceding row by \f$(a_k/b_{k−1})B_k\f$ and subtracting it. When it is in this form, the rates can be obtained directly through back substitution.

To reduce this matrix to the upper triangular form, the value of the variables as they are in the tridiagonal matrix form must first be stored. This is done by creating a set which holds the value of the above variables at each \f$k\f$ instance. Their value at each \f$k\f$ instance can be found using Equation 23-26. To reflect the value of the variables when the matrix is reduced to the upper triangular form, the \f$b\f$ and \f$d\f$ variable at each \f$k\f$ instance (row) can be updated using the following substitution for \f$3\leq k \leq N-1\f$:

\f[
    b_k → \frac{b_k - c_{k-1}a_k}{b_{k-1}} \tag*{28}
\f]
\f[
    \vec{d}_k → \vec{d}_k - (a_k/b_{k-1})B_k\cdot\vec{d}_{k-1}
\f]

Now that the stored values of the variables at each \f$k\f$ instance are updated to reflect the upper triangular matrix equation, the rates can be obtained directly by back substitution using:
\f[
    \vec{\omega}_{N-1} = \frac{\vec{d}_{N-1}}{b_{N-1}} \tag*{29}
\f]

and for \f$N - 2 \geq k \geq 2\f$:

\f[
    \vec{\omega}_k = \frac{\vec{d}_k - c_kB^{-1}_{k+1}\cdot\vec{\omega}_{k+1}}{b_k}
\f]

It is important that this back substitution is done in reverse \f$k\f$ order as each rate depends on the rate at a higher \f$k\f$ value as seen above.

Repeat this process of finding the matrix variables and calculating the angular rate from them until the difference between the current and previous iteration's values of the intermediate angular rate vector is less that a prescribed tolerance, \f$\epsilon_c\f$.

The difference is calculated as the L2 Norm using the following equation:
\f[
    \epsilon = \sqrt{\sum_{k=2}^{N-1}|\vec{\omega}_k-\vec{\omega}_k^{prev}|^2}\tag*{30}
\f]

Since \f$\vec{\omega}_1\f$ and \f$\vec{\omega}_N\f$ are given to the algorithm as \f$\vec{\omega}_{initial}\f$ and \f$\vec{\omega}_{final}\f$, once the intermediate rates are found using the above method, all rates at the input instances are known.  
<br>

#### Summary of getRates() algorithm
Within a loop that iterates until convergence (the difference is less than \f$\epsilon_c\f$) or until the maximum number of iterations, \f$m\f$, is exceeded:
* Find the coefficients of the matrix equation when it is in the upper triangular form - call `setRatesCoeffs()`
    * Determine the value of the coefficients in the original tridiagonal matrix using Equation 23-26 - calls `getRatesCoeffA()`, `getRatesCoeffB()`, `getRatesCoeffC`, `getRatesCoeffD()` for each intermediate instance, \f$k\f$
        * Determine the non-linear component of the angular rate vector using Equation 21. This is required to find the \f$d\f$ coefficient for each intermediate instance, \f$k\f$ - calls `getNonLinearRateTerm()`
        * Calls `bDotX()` and `bInvDotX()` to compute \f$B_2\cdot\vec{\omega}_{initial} \f$ and \f$B^{-1}_N\cdot\vec{\omega}_{final} \f$ required to find \f$\vec{d}_2\f$ and \f$\vec{d}_{N-1}\f$ respectively.
    * Determine the value of the coefficients when the matrix has been reduced to the upper triangular form using Equation 28
        * Calls `bDotX()`
* Solve for \f$\vec{\omega}_k\f$ using Equation 29 for all \f$k\f$, where \f$2 \leq k \leq N-1\f$.
    * Calls `bInvDotX()`
* Determine if the rates solution has converged using Equation 30.  
<br>

### Index explanation
In the paper and the above documentation, there are a lot of equations that use the value of variables at intermediate instance \f$k\f$. However, a lot of the variables represent behaviors over intervals, such as \f$\hat{e}\f$ or \f$\Delta\theta\f$. To make sure the indices within the equations in the C++ code match up, the sets that store the interval variables are made to be size N rather than size N-1 and are set to 0 or undefined at index 0. In this way, the interval values are stored at the index that represents the end of the interval (since \f$\hat{e}\f$ and \f$\Delta\theta\f$ are defined at \f$t_{end}\f$ of an interval as seen in Equation 17). Likewise, there are equations that use the interval index, \f$i\f$, which references both interval variables like the polynomial coefficients, and instance variables like the orientationPoints which are only defined at the start or end of an interval. Because this can be somewhat confusing and compounded by the change to 0-indexing in C++, the table below shows the value of these variables at the different indices, \f$k\f$ and \f$i\f$, for an example with 4 input Orientations.

<div style="text-align: center;">
<img src="https://cernbox.cern.ch/remote.php/dav/public-files/MzJ8hTQtQRAIH84/IndexExplanation.png?scalingup=0&preview=1&a=1&c=136054739199590400%3A0637b3e0&x=3840&y=3840" alt="Image" style="width:80%;height:auto;" />
</div>
