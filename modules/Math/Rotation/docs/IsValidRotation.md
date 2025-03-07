@ingroup group_rotation_is_valid_rotation

When using quaternion or matrix, input parameters are checked if they are,
respectively, a unitary quaternion or a rotation matrix up to a specified accuracy.
The default accuracy that is used, when accuracy parameter is not provided explicitely is 1e-12.

### Quaternion

Quaternion is checked, if it is a unitary quaternion up to the specified accuracy, by
checking whether (\f$q\f$ for the quaternion being checked, \f$\epsilon\f$ for the specified accuracy):
\f[
||q| - 1| < \epsilon.
\f]

### Matrix

Matrix is checked, if it is a rotation matrix up to the specified accuracy, by
checking whether (\f$M\f$ for the matrix being checked, \f$\epsilon\f$ for the specified accuracy,
\f$I\f$ for the identity matrix):
\f[
|MM^T - I|_{\infty} < \epsilon.
\f]
which checks if matrix is orthogonal up to the accuracy, and whether
\f[
|\det(M) - 1| < \epsilon
\f]
which cheks if matrix has determinant 1 up to the accuracy.
For the matrix \f$A\f$, of dimension \f$n\f$ infinity norm \f$|A|_\infty\f$ is defined as:
\f[
|A|_\infty = \max_{i, j \in \{1, \cdots, n\}} A_{i,j},
\f]
so as the maximum of all coefficients of A.
It was choosen as it is the most computationally effective and as
[all norms in finite dimensional spaces are equivalent](https://en.wikipedia.org/wiki/Norm_(mathematics)#Equivalent_norms).
