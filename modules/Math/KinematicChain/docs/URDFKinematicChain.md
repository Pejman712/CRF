@ingroup group_urdf_kinematic_chain

This implementation of IKinematicChain takes URDF files as a description of robots
topology and geometry.

<table>
<caption id="multi_row">Contributers summarize</caption>
<tr>    <th>@Name      <th>Role     <th>Company             <th>Year
<tr><td>@Ante Marić    <td>Author  <td>CERN - BE/CEM/MRO       <td>2022
<tr><td>@Bartosz Sójka     <td>Author  <td>CERN - BE/CEM/MRO       <td>2023
</table>

### URDF files

URDF file is an xml file describing the kinematic chain(s) of the robot.
We use one URDF file to describe the robot and one optional, separate
URDF file to describe a tool attached to the end effector of the robot.

For the conventions we follow and general rules of making URDF files, we
refer to
[URDF's guideline](https://confluence.cern.ch/pages/viewpage.action?pageId=422151198).

From now on, unless explicitly stated otherwise, we will assume, that the URDF is following
the conventions described in above document as 'necessary'.

### User's guide

#### Constructors

There is only one constructor with the three arguments:
    - path to the robot urdf
    - name of the link, which parent joint is leafJoint --
    i.e. the last joint in the kinematic chain that is always of the fixed type
    (in the case of the mobile platform, this argument can take any value,
    we recommend to pass "" then)
    It is defualted to the value "leafLink" which is a recommended name for that link
    - path to the tool urdf -- this is an optional argument when the tool is present, it defaults to "", representing no tol usage

Constructor checks the validity (by conventions from
[URDF's guideline](https://confluence.cern.ch/pages/viewpage.action?pageId=422151198))
and parses the URDF building the model of the kinematic chain.

Fixed joints are not counted as a joints after parsing. They do not affect the number of joint
and they cannot be accessed.

The initial joint positions are set to 0.0 for every joint.

After constructing there is no further initialisation needed and the class is ready to provide the geometrical data about the kinematic chain.

#### Setters

There is only one setter -- to set the joint positions.
It updates all of the geometrical data inside the class.
The length of the JointPositions should take wheels into account, even if the values
for the wheels don't matter at all in the calculations.
For example, for a robotic platform with the 6 DoF arm, JointPositions of size 10 should
be provided.

#### Getters

There are three getters, dependent of the type of the returned data:
    - axis
    - translation
    - rotation

Getters take as a argument an corresponding enum class:
    - [Axes](@ref crf::math::kinematicchain::Axes)
    - [Translations](@ref crf::math::kinematicchain::Translations)
    - [Rotations](@ref crf::math::kinematicchain::Rotations)

That encodes which exactly data should be returned, according to the convention wwriten in
the [Convention of naming geometrical data](@ref group_group_kinematic_chain#convention_of_naming_geometrical_data).

#### Compute functions

Function combine setter and getter, takes both arguments for the setter
and for the getter, updates all the
neccesary values inside the class and returns requested data.

### Developer's Guide

#### Details of the implementation

This is an non exostive list of quirkks and implementations details bout the URDFKinematicChain module.

URDF files are parsed as xml files by
[urdf parser](https://wiki.ros.org/urdf_parser).

The link of the name specified in 'endEffectorName' argument
is searched for.
It needs to be present, otherwise construcor will throw an exception.

From that link, the parent operator is repedately taken, until there
is a link with no parent joint.
This should be the 'rootLink' in properly prepared urdf.

The links with names consisting of the word "Wheel" are searched for.
Based on this, the number of wheels is determined.

All the links with "Wheel" in the name are checked if they
follow the naming convention. If yes, the platform is parsed,
if no, the error is thrown.



#### Further directions

Adding functionality for the multiple end effectors

Integrating most closely with new ypes (maybe returning TaskPose object?)

Function setting joint positions tries to update as little geometrical data as possible,
however, it is not yet perfect. It does not take into account that in the case of prismatic
joints, orientations of subsequent joints won't be affected and recalculates them anyway.
But tests should be performed if it is really needed.

Changing the scheme for description types from 'arm', 'combined', 'platform', to variables
'has arm', 'has platform' as this is the thing that is usually checked.
