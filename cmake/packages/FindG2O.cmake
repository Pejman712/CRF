# FindG2O.cmake
# The following standard variables get defined:
#  - G2O_FOUND:        TRUE if G2O is found.
#  - G2O_INCLUDE_DIRS: Include directories for G2O.
#  - G2O_STUFF_LIBRARY
#  - G2O_CORE_LIBRARY
#  - G2O_CLI_LIBRARY
#  - G2O_SOLVER_CHOLMOD
#  - G2O_SOLVER_CSPARSE
#  - G2O_SOLVER_CSPARSE_EXTENSION
#  - G2O_SOLVER_DENSE
#  - G2O_SOLVER_PCG
#  - G2O_SOLVER_SLAM2D_LINEAR
#  - G2O_SOLVER_STRUCTURE_ONLY
#  - G2O_SOLVER_EIGEN
#  - G2O_TYPES_DATA
#  - G2O_TYPES_ICP
#  - G2O_TYPES_SBA
#  - G2O_TYPES_SCLAM2D
#  - G2O_TYPES_SIM3
#  - G2O_TYPES_SLAM2D
#  - G2O_TYPES_SLAM3D

# Find the header files
find_path(G2O_INCLUDE_DIR NAMES g2o/core/base_vertex.h
                          PATHS ${G2O_ROOT}/include
                                $ENV{G2O_ROOT}/include
                                $ENV{G2O_ROOT}
                                /usr/local/include
                                /usr/include
                                /opt/local/include
                                /sw/local/include
                                /sw/include
                          NO_DEFAULT_PATH
)

# Macro to unify finding both the debug and release versions of the libraries; this is adapted from
# the OpenSceneGraph FIND_LIBRARY macro.
macro(find_g2o_library MYLIBRARY MYLIBRARYNAME)
    find_library("${MYLIBRARY}_DEBUG" NAMES "g2o_${MYLIBRARYNAME}_d"
                                      PATHS ${G2O_ROOT}/lib/Debug
                                            ${G2O_ROOT}/lib
                                            $ENV{G2O_ROOT}/lib/Debug
                                            $ENV{G2O_ROOT}/lib
                                      NO_DEFAULT_PATH
    )
    find_library("${MYLIBRARY}_DEBUG" NAMES "g2o_${MYLIBRARYNAME}_d"
                                      PATHS ~/Library/Frameworks
                                            /Library/Frameworks
                                            /usr/local/lib
                                            /usr/local/lib64
                                            /usr/lib
                                            /usr/lib64
                                            /opt/local/lib
                                            /sw/local/lib
                                            /sw/lib
    )
    find_library(${MYLIBRARY} NAMES "g2o_${MYLIBRARYNAME}"
                              PATHS ${G2O_ROOT}/lib/Release
                                    ${G2O_ROOT}/lib
                                    $ENV{G2O_ROOT}/lib/Release
                                    $ENV{G2O_ROOT}/lib
                              NO_DEFAULT_PATH
    )
    find_library(${MYLIBRARY} NAMES "g2o_${MYLIBRARYNAME}"
                              PATHS ~/Library/Frameworks
                                    /Library/Frameworks
                                    /usr/local/lib
                                    /usr/local/lib64
                                    /usr/lib
                                    /usr/lib64
                                    /opt/local/lib
                                    /sw/local/lib
                                    /sw/lib
    )
    if(NOT ${MYLIBRARY}_DEBUG)
        if(MYLIBRARY)
            set(${MYLIBRARY}_DEBUG ${MYLIBRARY})
        endif(MYLIBRARY)
    endif( NOT ${MYLIBRARY}_DEBUG)
endmacro(find_g2o_library)

# Find the core elements
find_g2o_library(G2O_STUFF_LIBRARY stuff)
find_g2o_library(G2O_CORE_LIBRARY core)

# Find the CLI library
find_g2o_library(G2O_CLI_LIBRARY cli)

# Find the pluggable solvers
find_g2o_library(G2O_SOLVER_CHOLMOD solver_cholmod)
find_g2o_library(G2O_SOLVER_CSPARSE solver_csparse)
find_g2o_library(G2O_SOLVER_CSPARSE_EXTENSION csparse_extension)
find_g2o_library(G2O_SOLVER_DENSE solver_dense)
find_g2o_library(G2O_SOLVER_PCG solver_pcg)
find_g2o_library(G2O_SOLVER_SLAM2D_LINEAR solver_slam2d_linear)
find_g2o_library(G2O_SOLVER_STRUCTURE_ONLY solver_structure_only)
find_g2o_library(G2O_SOLVER_EIGEN solver_eigen)

# Find the predefined types
find_g2o_library(G2O_TYPES_DATA types_data)
find_g2o_library(G2O_TYPES_ICP types_icp)
find_g2o_library(G2O_TYPES_SBA types_sba)
find_g2o_library(G2O_TYPES_SCLAM2D types_sclam2d)
find_g2o_library(G2O_TYPES_SIM3 types_sim3)
find_g2o_library(G2O_TYPES_SLAM2D types_slam2d)
find_g2o_library(G2O_TYPES_SLAM3D types_slam3d)

# G2O solvers declared found if we found at least one solver
set(G2O_SOLVERS_FOUND "NO")
if(G2O_SOLVER_CHOLMOD OR G2O_SOLVER_CSPARSE OR G2O_SOLVER_DENSE OR G2O_SOLVER_PCG OR G2O_SOLVER_SLAM2D_LINEAR OR G2O_SOLVER_STRUCTURE_ONLY OR G2O_SOLVER_EIGEN)
    set(G2O_SOLVERS_FOUND "YES")
endif()

# G2O itself declared found if we found the core libraries and at least one solver
set(G2O_FOUND "NO")
if(G2O_STUFF_LIBRARY AND G2O_CORE_LIBRARY AND G2O_INCLUDE_DIR AND G2O_SOLVERS_FOUND)
    set(G2O_FOUND "YES")
endif()
