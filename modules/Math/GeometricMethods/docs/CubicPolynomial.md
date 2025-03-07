@ingroup group_cubic_polynomial

This class generates a function that crosses a set of points at a specific range. It needs to receive through the constructor the next variables:

* A vector of the points to cross
* A vector of the range at which these points should be crossed (NOTE: these two vectors must be the same size!)
* The value of the first derivative at the starting range
* The value of the first derivative at the final range

If the function is evaluated outside the range it will remain constant, it will keep the initial and ending derivatives and move linearly with them.

<table>
<caption id="multi_row">Contributers summarize</caption>
<tr>    <th>@Name      <th>Role     <th>Company             <th>Year
<tr><td>@Jorge Playan Garai     <td>Author  <td>CERN - EN/SMM/MRO       <td>2022
</table>

An example of this function can be observed in the next image:

<img src="https://codimd.web.cern.ch/uploads/upload_bbef4e73085809a26ad8f8da17c8f259.png"  width="60%">

In order to calculate this function, we use the cubic polynomial equation:
~~~~~~~~~~~~~~~~~~~~~~~~{.py}
y(x)=a1+a2∗x+a3∗x2+a4∗x3
~~~~~~~~~~~~~~~~~~~~~~~~
This equation has four unknowns (a_1 ... a_4). In order to obtain these four parameters we need to place four constraints into the equation. These are:
~~~~~~~~~~~{.c}
y(t0)=P0
y(tf)=Pf˙
y(t0)=˙P0
˙y(tf)=˙Pf
~~~~~~~~~~~

These four restrictions will force the equation to go from: ```P_0 in t_0 to P_f in t_f.```

However, we want to connect several point smoothly and create a function that goes through all of them, not only two. For this we need to implement a cubic polynomial to connect each pair of points.
For an N number of points we need ```N-1``` equations with ```4(N-1)``` total variables.
In order to solve this equations we will need the same number of restrictions, ```4(N-1)```
We know that at each specific time we need to be in the correspondent point on both extremes

~~~~~~~~~~~~~
yn(tn)=Pn
yn(tn−1)=Pn−1
~~~~~~~~~~~~~

We also know the start and end first derivative values:
~~~~~~~~~~
˙y(t0)=˙P0
˙y(tf)=˙Pf
~~~~~~~~~~

This gives us ```2(n-1) + 2``` equations.
We can add some extra restrictions:
At the connection between cubic equations, we want the first and second derivatives to match each other:
~~~~~~~~~~~~~~~~~
˙yn−1(tn)=˙yn(tn)
¨yn−1(tn)=¨yn(tn)
~~~~~~~~~~~~~~~~~

Including these restrictions we can add ```2(N-2)```. Now we have a total of ```4(n-1)``` restrictions for the same number of unknowns.
With these equations we can generate a sparse matrix and solve it to obtain the values of every equation. After that, once a value of x is received we calculate which equation is in range for it and return the correspondent value;

---

In the non-linear case, there is an extra incognita ```t_f```

Now we have 5 unknowns ```(a_1 ... a_4, t_f)``` and we have 7 restrictions

~~~~~~~~~~~
y(t_0) = Y_0
y(t_f) = Y_f
y'(t_0) = Y_d0
y'(t_f) = Y_df
|y'(t_dmax)| < Y_dmax
|y``(t_0)| < Y_ddmax
|y``(t_f)| < Y_ddmax
~~~~~~~~~~~

The last 2 constraints are the maximum acceleration limits. Since the third derivative has the form:

~~~~~~~~~~~
y(t) = 2*a3 + 6*a4∗t
~~~~~~~~~~~

Which transform into a line. The maximum an minimum will be at the end of the ranges (```t_0```, ```t_f```)

These contrainsts translates into the following system of equations:

~~~~~~~~~~~
Y_0 = a1 + a2∗t_0 + a3∗t_0^2 + a4∗t_0^3
Y_f = a1 + a2∗t_f + a3∗t_f^2 + a4∗t_f^3
Y_d0 = a2 + 2*a3∗t_0 + 3*a4∗t_0^2
Y_df = a2 + 2*a3∗t_f + 3*a4∗t_f^2
Y_dmax > a2 + 2*a3∗t_dmax + 3*a4∗t_dmax^2
Y_ddmax > 2*a3 + 6*a4∗t_0
Y_ddmax > 2*a3 + 6*a4∗t_f
~~~~~~~~~~~

Where the maximum ```t_dmax``` is the point that makes the following derivative 0

~~~~~~~~~~~
0 = 2*a3 + 6*a4∗t_dmax
t_dmax = -2*a3 / (6*a4)
t_dmax = -a3 / (3*a4)
~~~~~~~~~~~

~~~~~~~~~~~
Y_0 = a1 + a2∗t_0 + a3∗t_0^2 + a4∗t_0^3
Y_f = a1 + a2∗t_f + a3∗t_f^2 + a4∗t_f^3
Y_d0 = a2 + 2*a3∗t_0 + 3*a4∗t_0^2
Y_df = a2 + 2*a3∗t_f + 3*a4∗t_f^2
Y_dmax > a2 + 2*a3∗(-a3 / (3*a4)) + 3*a4∗(-a3 / (3*a4))^2
Y_ddmax > 2*a3 + 6*a4∗t_0
Y_ddmax > 2*a3 + 6*a4∗t_f
~~~~~~~~~~~

Since ```t_f``` is an unknown, this is a non-linear system of equations and we need a numerical solver. For this purpose, we use [NLopt](https://nlopt.readthedocs.io/en/latest/).

In Nlopt there are multitude of methods to solve this equations. Not all have been tested but among the ones tested, the best results where obtained with:
- GN_ISRES: Takes a while but finds the solution consistently
- LN_COBYLA: Very fast but not as reliable

Since GN_ISRES was the only one providing consistent solutions, it was the one selected for this task.
