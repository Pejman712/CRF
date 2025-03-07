/* © Copyright CERN 2023. All rights reserved. This software is released under a
 * CERN proprietary software license. Any permission to use it shall be granted
 * in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Ante Marić CERN BE/CEM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

namespace crf::math::kinematicchain {

/**
 * @ingroup group_kinematic_chain_geometric_data
 * @brief enum class of all implemented axes\n
 *
 */
enum class Axes {
    /**
     * @brief Axis expressed in the (I)nertial frame,
     * of the (W)heel of a specified number.
     */
    IW,

    /**
     * @brief Axis expressed in the (W)heels frame,
     * of the (W)heel of a specified number.
     */
    WW,

    /**
     * @brief Axis expressed in the (I)nertial frame,
     * of the (J)oint of a specified number.
     */
    IJ,

    /**
     * @brief Axis expressed in the (J)oints frame,
     * of the (J)oint of a specified number.
     */
    JJ,
};

/**
 * @ingroup group_kinematic_chain_geometric_data
 * @brief enum class of all implemented translations\n
 *
 */
enum class Translations {
    /**
     * @brief Translation expressed in the (I)nertial frame,
     * from the (I)nertial frame to the (W)heel of a specified number.
     */
    IIW,

    /**
     * @brief Translation expressed in the (I)nertial frame,
     * from the (I)nertial frame to the (J)oint of a specified number.
     */
    IIJ,

    /**
     * @brief Translation expressed in the (I)nertial frame,
     * from the (I)nertial frame to the (E)nd effector.
     */
    IIE,

    /**
     * @brief Translation expressed in the (I)nertial frame,
     * from the (P)arent of the joint to the (J)oint of a specified number.
     */
    IPJ,

    /**
     * @brief Translation expressed in the (I)nertial frame,
     * from the (J)oint of a specified number to the (E)nd effector.
     */
    IJE,

    /**
     * @brief Translation expressed in the (P)arent of the joint frame,
     * from the (P)arent of the joint to the (J)oint of a specified number.
     */
    PPJ,

    /**
     * @brief Translation expressed in the (P)arent of the joint frame,
     * from the (P)arent of the joint to the (J)oint of a specified number.
     */
    JJE,

    /**
     * @brief Translation expressed in the (L)ast joint frame,
     * from the (L)ast joint to the (E)nd effector.
     */
    LLE,
};

/**
 * @ingroup group_kinematic_chain_geometric_data
 * @brief enum class of all implemented rotations\n
 *
 */
enum class Rotations {
    /**
     * @brief Rotation expressed in the (I)nertial frame,
     * from the (I)nertial frame to the (W)heel of a specified number.
     */
    IIW,

    /**
     * @brief Rotation expressed in the (I)nertial frame,
     * from the (I)nertial frame to the (J)oint of a specified number.
     */
    IIJ,

    /**
     * @brief Rotation expressed in the (I)nertial frame,
     * from the (I)nertial frame to the (E)nd effector.
     */
    IIE,

    /**
     * @brief Rotation expressed in the (P)arent of the joint frame,
     * from the (P)arent of the joint to the (J)oint of a specified number.
     */
    PPJ,

    /**
     * @brief Rotation expressed in the (L)ast joint frame,
     * from the (L)ast joint to the (E)nd effector.
     */
    LLE,
};

}  // namespace crf::math::kinematicchain
