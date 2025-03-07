/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Adrien Luthi CERN EN/SMM/MRO 2023
 *
 *  ===============================================================================================
 */

#pragma once

enum class CavityOrientation {
    Undefined = 0,
    /**
     * @brief Based on the beam direction of the cavity definition, this enum defines how the
     *        cavity is fixed on the system. IO stand for Input-Output (of the beam in the cavity).
     *        Therefore, if the cavity is fixed with the orientation IO, the robotics arm enters it
     *        on the Input side.
     */
    IO = 1,
    /**
     * @brief Based on the beam direction of the cavity definition, this enum defines how the
     *        cavity is fixed on the system. OI stand for Output-Input (of the beam in the cavity).
     *        Therefore, if the cavity is fixed with the orientation OI, the robotics arm enters it
     *        on the Output side.
     */
    OI = 2
};

enum class CavityType {
    Undefined = 0,
    /**
     * @brief The cavity installed can be the 400MHz (LHC) one.
     *
     */
    LHC = 1,
    /**
     * @brief The cavity installed can be the 704 MHz (5-cells) one.
     *
     */
    FiveCells = 2,
    /**
     * @brief The cavity installed can be the 1.3 GHz (FCC) one.
     *
     */
    FCC = 3
};
