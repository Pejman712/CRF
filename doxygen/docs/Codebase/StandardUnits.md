# Standard Units {#standard_units}

The Standard units used in the CRF follow the International System of Units (SI).

The International System of Units (SI) is a globally recognized framework for measurement. It provides a consistent and coherent system of units for various physical quantities. This file outlines the units used in SI for different measurements, including some derived units that are relevant to robotics.

Each of these units can be modified by factors of 10. In these cases, the base unit is preferred. The only exception will be the kilogram, which will be used instead of the gram as defined in the SI. Exceptions shall be justified and the units properly stated.

## Length

- Meter (_m_): The SI base unit for length. It is defined as the distance traveled by light in a vacuum during a specific time interval.

## Mass

- Kilogram (_kg_): The SI base unit for mass. It is defined as the mass of the International Prototype of the Kilogram, a platinum-iridium cylinder kept at the International Bureau of Weights and Measures.
- Gram (_g_): Equal to 0.001 kilograms.
- Tonne (_t_): Equal to 1000 kilograms.

Among all of these, the kilogram is to be preferred and used by default unless specifically stated otherwise and properly justified.

## Time

- Second (_s_): The SI base unit for time. It is defined as the duration of 9,192,631,770 periods of radiation corresponding to the transition between two hyperfine levels of the ground state of the cesium-133 atom.

## Electric Current

- Ampere (_A_): The SI base unit for electric current. It is defined as the constant current that, if maintained in two parallel conductors of infinite length and negligible cross-section, would produce a force between the conductors of exactly 2 x 10^-7 newton per meter of length.

## Temperature

- Kelvin (_K_): The SI base unit for temperature. It is defined as 1/273.16 of the thermodynamic temperature of the triple point of water.

## Amount of Substance

- Mole (_mol_): The SI base unit for the amount of substance. It is defined as the amount of substance that contains exactly 6.02214076 x 10^23 elementary entities.

## Luminous Intensity

- Candela (_cd_): The SI base unit for luminous intensity. It is defined as the luminous intensity, in a given direction, of a source that emits monochromatic radiation of frequency 540 x 10^12 hertz and that has a radiant intensity in that direction of 1/683 watt per steradian.

## Derived Units Relevant to Robotics and the CRF

- Velocity (_m/s_): Derived from the base units of length and time. Represents the rate of change of position.
- Acceleration (_m/s^2_): Derived from the base units of length and time. Represents the rate of change of velocity.
- Force (_N_): Derived from the base units of mass, length, and time. Represents the interaction between two objects and their motion.
- Torque (_N·m_): Derived from the base units of force and length. Represents the tendency of a force to rotate an object about an axis.
- Power (_W_): Derived from the base units of force and length. Represents the rate at which work is done or energy is transferred.
- Energy (_J_): Derived from the base units of force and length. Represents the capacity to do work.
- Pressure (_Pa_): Derived from the base units of force and length. Represents the force exerted on a surface per unit area.
- Angle (_rad_): The SI unit for angles. It represents the ratio of the length of an arc of a circle to the radius of the circle.
- Angular Velocity (_rad/s_): Derived from the base units of angle and time. Represents the rate of change of angular displacement.
- Angular Acceleration (_rad/s^2_): Derived from the base units of angle and time. Represents the rate of change of angular velocity.

---

Please note that this list is not exhaustive, and there may be other derived units and prefixes that are relevant to specific robotics applications. As a general rule, always stick to the SI unless explicitly stated.
