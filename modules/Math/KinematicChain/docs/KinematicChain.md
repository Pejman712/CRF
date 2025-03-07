@ingroup group_kinematic_chain

The kinematic chain module consists of interfaces and implementations of kinematic chains --
objects that represents geometrical data of robots e.g. what are translations and rotations
between different joints of the robot when the joints are in a particular position.

<table>
<caption id="multi_row">Contributers summarize</caption>
<tr>    <th>@Name      <th>Role     <th>Company             <th>Year
<tr><td>@Ante Marić    <td>Author  <td>CERN - BE/CEM/MRO       <td>2022
<tr><td>@Bartosz Sójka     <td>Author  <td>CERN - BE/CEM/MRO       <td>2023
</table>

### Architecture

For now there is only one interface -- IKinematicChain and one implementation of this interface --
URDFKinematicChain.

\anchor convention_of_naming_geometrical_data

### Convention of naming geometrical data

Geometric data description codes:
    - XYZ - expressed in the frame of X, from Y to Z;
    - XY - expressed in the frame X, something of Y;
    - Xt_YZ - expressed in the frame of X, object of type t, from Y, to Z;
    - Xt_Y - expressed in the frame of X, object of type t associated with Y;
    - I - inertial frame,
    - E - end effector,
    - J - joint of the specified number,
    - W - wheel of the specified number,
    - L - last joint before the end effector,
    - P - parent,
    - C - child;
    - n - axis,
    - r - translation,
    - R - rotation;
