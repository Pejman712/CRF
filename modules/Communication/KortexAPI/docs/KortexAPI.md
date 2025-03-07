@ingroup group_kortexapi

### SOEM Linking Error

If you've arrived to this page it probably means that you saw a message in the CMake of this module directing you towards this file or that you've encountered an error in the form of:

```sh
/usr/bin/ld: /opt/Kortex/lib/release/libKortexApiCpp.a(KeEthercatMaster.cpp.o):(.data+0x0): multiple definition of `ecx_context'; /usr/local/lib/libsoem.a(ethercatmain.c.o):(.data.rel+0x0): first defined here
/usr/bin/ld: /opt/Kortex/lib/release/libKortexApiCpp.a(KeEthercatMaster.cpp.o): in function `ecx_setupnic':
KeEthercatMaster.cpp:(.text+0xaf0): multiple definition of `ecx_setupnic'; /usr/local/lib/libsoem.a(nicdrv.c.o):nicdrv.c:(.text+0x3d): first defined here
/usr/bin/ld: /opt/Kortex/lib/release/libKortexApiCpp.a(KeEthercatMaster.cpp.o): in function `ecx_closenic':
KeEthercatMaster.cpp:(.text+0x1480): multiple definition of `ecx_closenic'; /usr/local/lib/libsoem.a(nicdrv.c.o):nicdrv.c:(.text+0x59a): first defined here
/usr/bin/ld: /opt/Kortex/lib/release/libKortexApiCpp.a(KeEthercatMaster.cpp.o): in function `ec_setupheader':
KeEthercatMaster.cpp:(.text+0x14b0): multiple definition of `ec_setupheader'; /usr/local/lib/libsoem.a(nicdrv.c.o):nicdrv.c:(.text+0x601): first defined here
/usr/bin/ld: /opt/Kortex/lib/release/libKortexApiCpp.a(KeEthercatMaster.cpp.o): in function `ecx_getindex':
KeEthercatMaster.cpp:(.text+0x1500): multiple definition of `ecx_getindex'; /usr/local/lib/libsoem.a(nicdrv.c.o):nicdrv.c:(.text+0x6a8): first defined here
/usr/bin/ld: /opt/Kortex/lib/release/libKortexApiCpp.a(KeEthercatMaster.cpp.o): in function `ecx_setbufstat':
KeEthercatMaster.cpp:(.text+0x19b0): multiple definition of `ecx_setbufstat'; /usr/local/lib/libsoem.a(nicdrv.c.o):nicdrv.c:(.text+0x798): first defined here
/usr/bin/ld: /opt/Kortex/lib/release/libKortexApiCpp.a(KeEthercatMaster.cpp.o): in function `ecx_outframe':
KeEthercatMaster.cpp:(.text+0x19e0): multiple definition of `ecx_outframe'; /usr/local/lib/libsoem.a(nicdrv.c.o):nicdrv.c:(.text+0x7f6): first defined here
/usr/bin/ld: /opt/Kortex/lib/release/libKortexApiCpp.a(KeEthercatMaster.cpp.o): in function `ecx_outframe_red':
KeEthercatMaster.cpp:(.text+0x1a30): multiple definition of `ecx_outframe_red'; /usr/local/lib/libsoem.a(nicdrv.c.o):nicdrv.c:(.text+0x8b0): first defined here
/usr/bin/ld: /opt/Kortex/lib/release/libKortexApiCpp.a(KeEthercatMaster.cpp.o): in function `ec_setupnic':
KeEthercatMaster.cpp:(.text+0x1d30): multiple definition of `ec_setupnic'; /usr/local/lib/libsoem.a(nicdrv.c.o):nicdrv.c:(.text+0x11c8): first defined here
/usr/bin/ld: warning: size of symbol `ecx_port' changed from 52016 in /usr/local/lib/libsoem.a(ethercatmain.c.o) to 51944 in /opt/Kortex/lib/release/libKortexApiCpp.a(KeEthercatMaster.cpp.o)
```

The main point of this file is to explain why this error happens, and how it's solved. Bear in mind that I'm no expert in the topic but I'll point towards the links that helped me reach this conclusion.

Take all of the following info with a grain of salt and bear in mind that this is just a hypothesis. If more info is found please add it to this file.

#### Why it happens

The main problem relates to three of our modules which depend on external libraries:

- CANopenDrivers: Depends on SOEM
- EtherCATDevices (legacy): Depends on SOEM
- KortexAPI: Depends on Kortex

Most importantly, the compilation and linking on SOEM is done on our side while we only have the binaries from Kortex. As we can see in the previous linking errors, we get (among other errors):

```sh
/usr/bin/ld: /opt/Kortex/lib/release/libKortexApiCpp.a(KeEthercatMaster.cpp.o):(.data+0x0): multiple definition of `ecx_context'; /usr/local/lib/libsoem.a(ethercatmain.c.o):(.data.rel+0x0): first defined here
```

This says that the library Kortex and SOEM share symbol names. Particulary, it points towards the KeEthercatMaster.cpp file, a file from Kortex which we have no access to (again, we only get the binaries). Based on the name of the file and in the repeated symbol (ecx_context), we can infer that Kortex is using SOEM under-the-hood to follow the EtherCAT protocol, and a copy of SOEM came with their binaries which is now creating conflicts with our own copy.

Basically, the problem arises because one SOEM instance get's linked and then it tries to link the next one which has the same symbols defined (duh, it's the same library), and it complains that the functions have already been defined before so we can't use the same names.

#### Solution

After some intensive browsing, I stumbled into two quite helpful posts ([first](https://stackoverflow.com/questions/69889858/how-can-reordering-the-linked-libraries-fix-multiple-definitions-error) and [second](https://eli.thegreenplace.net/2013/07/09/library-order-in-static-linking)) which comment how the order of linking libraries actually influences the multiple definitions error.

As you can imagine, changing at which the library get compiled fixed the issue, thus KinovaGen3 should be compiled before CiA402Robot and EtherCATRobot.

I believe what's happening is the following:

- Kortex has a dependency in SOEM through the binaries.
- EtherCATDevices and CANopenDrivers are linking into SOEM

Both libraries have the following structure:

- Kortex -> (Kortex + SOEM): Non separable, binaries together, the parenthesis are written to represent this
- CANopenDrivers = CANopenDrivers + SOEM: linked together, can be separated, no parenthesis

If you link first Kortex you'll get:

- Kortex linked: (Kortex + SOEM)
- CANopenDrivers linked: CANopenDrivers
- SOEM linked: Already found the symbols for it, it does not get linked

This works, since the second linking of SOEM is repeated and can be separated and discarded. If we do it backwards:

- CANopenDrivers linked: CANopenDrivers
- SOEM linked: SOEM
- Kortex linked: (Kortex + SOEM): Now there's a problem, Kortex needs to be added so it links the full module since some symbols are not there and are needed, but with Kortex also comes another copy of SOEM that cannot be separated! Now we have multiple definitions of SOEM and thus the error

#### More info

Before this solution we contacted Kinova. They assured us these were legacy files that should be removed eventually as they don't control the motor through EtherCAT anymore. Hopefully, the files will be removed from Kortex in a future release and this won't be needed.
