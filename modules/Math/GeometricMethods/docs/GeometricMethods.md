@ingroup group_geometric_methods

The geometric methods module includes a collection of different algorithms that generate functions over a specified range (f(x)). These methods are later used for objective functions in inverse kinematics and for trajectory generation among other uses.

<table>
<caption id="multi_row">Contributers summarize</caption>
<tr>    <th>@Name      <th>Role     <th>Company             <th>Year
<tr><td>@Laura Rodrigo Perez    <td>Author  <td>CERN - EN/SMM/MRO       <td>2022
<tr><td>@Jorge Playan Garai     <td>Author  <td>CERN - EN/SMM/MRO       <td>2022
</table>

### Architecture

All the different implementations of geometric methods include a common interface. This interface includes two basic functions that any geometric method must implement. The general inheritance structure of these classes is the following:

![General Inheritance Structure](https://codimd.web.cern.ch/uploads/upload_db8eed4c9a2f92ed425765b7b6b337b0.png)
