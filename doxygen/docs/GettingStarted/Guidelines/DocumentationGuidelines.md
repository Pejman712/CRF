# Documentation Guidelines {#documentation_guidelines}

The purpose of this guideline is to advise users on how to correctly document new, old or update modules and/or implementations they are working on.

## Documentation for header files (.hpp)

When adding or working on current .hpp files, users will be required to categorise which module, submodule or implementation it belongs to. This is defined in the file [group_defs.dox](https://gitlab.cern.ch/mro/robotics/cernroboticframework/cpproboticframework/-/blob/master/doxygen/group_defs.dox).
Users will add the appropriate new group to this file.

Within the .hpp file users will, in addition to the **brief**, add the following to their file after the namespace delcartion:

```
/*
 * @ingroup group_new_group
 * @brief An example explanation of the following class.
 */
```

### Notes

Exclude .hpp files such as unit tests files and mock files as well as .cpp src files.

## Documentation for markdown files (.md)

Within each module, users will find a **docs** folder. This folder will contain all markdown files pertaining to the respective module, submodule and/or implementations.

The following are the guidelines users are required to follow when creating markdown files:

```
@ingroup group_new_group

The title of each markdown file should be the appropriate ingroup used in the .hpp file(s) instead of the title such as "(BotaEtherCAT {#bota_ethercat})."

###  Title levels

Subsequent titles or subtitle must only be made using titles of level 3 ### and 4 ####.

### References

References can be written like [this](@ref standard_units), and we can also refer to types with their namesapce crf::expected<T>.

### Equations
The equations can be written using the Latex syntax

\f[
    \label{eq:2ndDerivative}
	\ddot{\sigma} = \begin{cases}
        0                         & t<0\;
    	d''_{max} \sin(\omega t)^2  & t<\frac{\pi}{\omega}\:
		-d''_{max} \sin(\omega t)^2 & t<\frac{2\pi}{\omega}\,
		0                         & \mathrm{else},
	\end{cases}
\f]

```

### Supported Emojis

Found [here](https://github-emoji-picker.rickstaa.dev/)
