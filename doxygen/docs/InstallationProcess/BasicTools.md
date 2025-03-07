# Basic Tools {#basic_tools}

Once a minimal installation of the Linux system is done, some basic packages are required to be able to install many of the libraries used by the CERN Robotic Framework.

In the folder ./scripts/systemSetup there is a script installDependencies.sh. When run without arguments, it will print some help information. The arguments that can be provided are names of dependencies to be installed. The script is launched as follows:

````bash
./installDependencies BasicTools
````

This command requires sudo priviliges, so it will ask for the PC credentials. For more details on the utilization of the script execute:

````bash
./installDependencies help
````

After this is completed you can continue with the rest of the libraires installation in the page \ref compilation_and_execution

#### Compilers

| Name | Description | License |
| :-: | :-: | :-: |
| g++ | It is a GNU c++ compiler invocation command, which is used for preprocessing, compilation, assembly and linking of source code to generate an executable file. | |
| gcc | The GNU Compiler Collection includes front ends for C, C++, Objective-C, Fortran, Ada, Go, and D, as well as libraries for these languages (libstdc++,...). GCC was originally written as the compiler for the GNU operating system. | |

#### Assemblers

| Name | Description | License |
| :-: | :-: | :-: |
| NASM | The Netwide Assembler (NASM) is an assembler and disassembler for the Intel x86 architecture. It can be used to write 16-bit, 32-bit (IA-32) and 64-bit (x86-64) programs. | |

#### Languages

| Name | Description | License |
| :-: | :-: | :-: |
| Python | Python, the high-level, interactive object oriented language, includes an extensive class library with lots of goodies for network programming, system administration, sounds and graphics. | |

#### Build Processes

| Name | Description | License |
| :-: | :-: | :-: |
| Autoconf | It is an extensible package of M4 macros that produce shell scripts to automatically configure software source code packages. | |
| CMake | CMake is an open-source, cross-platform family of tools designed to build, test and package software. | |
| Make | The purpose of the make utility is to determine automatically which pieces of a large program need to be recompiled, and issue the commands to recompile them. | |

#### Version Control

| Name | Description | License |
| :-: | :-: | :-: |
| Git | Git is a free and open source distributed version control system designed to handle everything from small to very large projects with speed and efficiency. | |

#### Testing

| Name | Description | License |
| :-: | :-: | :-: |
| LCOV | LCOV is a graphical front-end for GCC's coverage testing tool gcov. It collects gcov data for multiple source files and creates HTML pages containing the source code annotated with coverage information. | |

#### Debugging

| Name | Description | License |
| :-: | :-: | :-: |
| GDB | GDB, the GNU Project debugger, allows you to see what is going on `inside' another program while it executes -- or what another program was doing at the moment it crashed. | |
| Valgrind | The Valgrind tool suite provides a number of debugging and profiling tools that help you make your programs faster and more correct. | |

#### Extra

| Name | Description | License |
| :-: | :-: | :-: |
| AutoGen | It is a tool designed to simplify the creation and maintenance of programs that contain large amounts of repetitious text. It is especially valuable in programs that have several blocks of text that must be kept synchronized. | |
| Htop | Htop is an interactive system-monitor process-viewer and process-manager. It is designed as an alternative to the Unix program top. | |
| SSH | The SSH protocol uses encryption to secure the connection between a client and a server. All user authentication, commands, output, and file transfers are encrypted to protect against attacks in the network. | |
| VIM | Vim is a highly configurable text editor built to make creating and changing any kind of text very efficient. It is included as "vi" with most UNIX systems and with Apple OS X. | |
