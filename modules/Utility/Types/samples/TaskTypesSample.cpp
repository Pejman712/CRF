/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Types/TaskTypes/TaskTypes.hpp"

using crf::math::rotation::Orientation;
using crf::utility::types::TaskPose;
using crf::utility::types::Vector6d;
using crf::utility::types::TaskVelocity;
using crf::utility::types::TaskAcceleration;
using crf::utility::types::TaskForceTorque;
using crf::utility::types::TaskSpace;
using crf::utility::types::TaskSpaceTangentDimension;

using crf::math::rotation::CardanXYZ;
using crf::math::rotation::EulerZXZ;

int main() {
    /**
     * TaskPose
    */

    /**
     * Identities in translation, all supported orientation representations
     * and homogeneous transformation matrix, as an examples
     * of objects from which TaskPose class can be constructed or their value can be set.
     */
    Eigen::Matrix4d homogeneousTransformationMatrix(Eigen::Matrix4d::Identity());
    Orientation orientation;
    Eigen::Vector3d position({0.0, 0.0, 0.0});
    Eigen::Quaterniond quaternion({1.0, 0.0, 0.0, 0.0});
    Eigen::Matrix3d matrix(Eigen::Matrix3d::Identity());
    Eigen::AngleAxisd angleAxis(1.0, Eigen::Vector3d({1.0, 0.0, 0.0}));
    CardanXYZ cardanXYZ({0.0, 0.0, 0.0});
    EulerZXZ eulerZXZ({0.0, 0.0, 0.0});

    /**
     * Default constructor.
     * Creates an identity task pose in the quaternion representation.
     */
    TaskPose taskPose0;

    /**
     * Constructors from different representations.
     * Create corresponding taskPose in corresponding representation.
     * Quaternion and matrix need to be, respectively, unitary quaternion and taskPose matrix.
     */
    TaskPose taskPose1(homogeneousTransformationMatrix);
    TaskPose taskPose1_0_1(position, orientation);
    TaskPose taskPose2(position, quaternion);
    TaskPose taskPose3(position, matrix);
    TaskPose taskPose4(position, angleAxis);
    TaskPose taskPose5(position, cardanXYZ);
    TaskPose taskPose6(position, eulerZXZ);
    /**
     * Constructors from homogeneous transformation matrix, position and quaternion, and
     * position and matrix can be set with the parameter accuracy,
     * adjusting how accurate homogeneous transformation matrix, quaternion or matrix should be to
     * representing the rotation in task pose.
     * The default is 1e-12.
    */
    TaskPose taskPose1_1(homogeneousTransformationMatrix, 1e-7);
    TaskPose taskPose2_1(position, quaternion, 1e-7);
    TaskPose taskPose3_1(position, matrix, 1e-7);
    /**
     * When constructing directly from numerical values, instead of writing:
    */
    TaskPose taskPose1_0_2(Eigen::Vector3d({0.0, 0.0, 0.0}), orientation);
    /**
     * Constructors involving position can be written as:
    */
    TaskPose taskPose1_0_3({0.0, 0.0, 0.0}, orientation);

    /**
     * Setters
     * Sets the value of the taskPose and changes to the representation of the parameter.
     * Quaternion and matrix need to be, respectively, unitary quaternion and rotation matrix.
    */
    taskPose1.setPosition(position);
    taskPose2.setOrientation(orientation);
    taskPose1.setEulerZXZ(eulerZXZ);
    taskPose2.setRotationMatrix(matrix);
    taskPose3.setAngleAxis(angleAxis);
    taskPose4.setCardanXYZ(cardanXYZ);
    taskPose5.setQuaternion(quaternion);
    taskPose6.setHomogeneousTransformationMatrix(homogeneousTransformationMatrix);
    /**
     * Setters from quaternion and matrix can be set with the parameter accuracy,
     * adjusting how accurate quaternion or matrix should be to representing the taskPose.
     * The default is 1e-12.
    */
    taskPose2.setRotationMatrix(matrix, 1e-7);
    taskPose4.setQuaternion(quaternion, 1e-7);
    taskPose6.setHomogeneousTransformationMatrix(homogeneousTransformationMatrix, 1e-7);

    /**
     * Getters
     * Gets desired representation regardless of the current representation of the TaskPose
     * object.
     * Const member functions. Does not change anything inside the class, nor the value,
     * nor the representation.
    */
    homogeneousTransformationMatrix = taskPose6.getHomogeneousTransformationMatrix();
    position = taskPose3.getPosition();
    orientation = taskPose4.getOrientation();
    matrix = taskPose3.getRotationMatrix();
    quaternion = taskPose2.getQuaternion();
    cardanXYZ = taskPose1.getCardanXYZ();
    angleAxis = taskPose4.getAngleAxis();
    eulerZXZ = taskPose5.getEulerZXZ();
    OrientationRepresentation orientationRepresentation1 = taskPose1.getOrientationRepresentation();

    /**
     * Print to stream operator. Prints taskPose in accordance to its representation.
    */
    std::cout << taskPose0 << std::endl;
    std::cout << taskPose1 << std::endl;
    std::cout << taskPose2 << std::endl;
    std::cout << taskPose3 << std::endl;
    std::cout << taskPose4 << std::endl;
    std::cout << taskPose5 << std::endl;
    std::cout << taskPose6 << std::endl;

    /**
     * TaskVelocity, TaskAcceleration, TaskForceTorques
    */

    double inputDouble = 1.7;
    double outputDouble = 0;
    /**
     * Sample std::array<double, 6> and Eigen::Vector<double, 6> from which
     * TaskVelocity, TaskAcceleration and TaskForceTorque can be constructed.
    */
    const Eigen::Vector<double, 6> eigenVector({3.14, 1.7, 0.5, 2.15, 43.44, -0.15});
    const std::array<double, 6> stdArrayDouble({3.14, 1.7, 0.5, 2.15, 43.44, -0.15});

    /**
     * Default constructors.
     * Creates an object with [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] vector.
    */
    TaskVelocity taskVelocity0;
    TaskAcceleration taskAcceleration0;
    TaskForceTorque taskForceTorque0;

    /**
     * Constructors from Eigen::Vector<double, 6>, std::array<double, 6> and initialiser_list
    */
    TaskVelocity taskVelocity1(eigenVector);
    TaskAcceleration taskAcceleration1(eigenVector);
    TaskForceTorque taskForceTorque1(eigenVector);
    TaskVelocity taskVelocity2(stdArrayDouble);
    TaskAcceleration taskAcceleration2(stdArrayDouble);
    TaskForceTorque taskForceTorque2(stdArrayDouble);
    TaskVelocity taskVelocity3({2.17, 3.15, 0.7, 1.7, 0.5, 2.15});
    TaskAcceleration taskAcceleration3({2.17, 3.15, 0.7, 1.7, 0.5, 2.15});
    TaskForceTorque taskForceTorque3({2.17, 3.15, 0.7, 1.7, 0.5, 2.15});

    /**
     * Assignment operators from Eigen::Vector<double, 6>, std::array<double> and initialiser_list
    */
    taskVelocity1 = eigenVector;
    taskAcceleration1 = eigenVector;
    taskForceTorque1 = eigenVector;
    taskVelocity1 = stdArrayDouble;
    taskAcceleration1 = stdArrayDouble;
    taskForceTorque1 = stdArrayDouble;
    taskVelocity1 = {2.17, 3.15, 0.7, 1.7, 0.5, 2.15};
    taskAcceleration1 = {2.17, 3.15, 0.7, 1.7, 0.5, 2.15};
    taskForceTorque1 = {2.17, 3.15, 0.7, 1.7, 0.5, 2.15};

    /**
     * Index operator.
    */
    taskVelocity1[4] = inputDouble;
    taskAcceleration1[4] = inputDouble;
    taskForceTorque1[4] = inputDouble;
    outputDouble = taskVelocity1[4];
    outputDouble = taskAcceleration1[4];
    outputDouble = taskForceTorque1[4];

    /**
     * Size function. Always returns 6.
    */
    taskVelocity1.size();
    taskAcceleration1.size();
    taskForceTorque1.size();

    /**
     * Raw function, returns Eigen::Vector<double, 6> with the coordinates.
    */
    taskVelocity1.raw();
    taskAcceleration1.raw();
    taskForceTorque1.raw();

    /**
     * Print to stream operator.
    */
    std::cout << taskVelocity1 << std::endl;
    std::cout << taskAcceleration1 << std::endl;
    std::cout << taskForceTorque1 << std::endl;

    /**
     * TaskSpace
    */

    bool inputBool = true;
    bool outputBool = false;
    /**
     * Sample std::map<TaskSpaceTangentDimension, bool>, std::array<bool, 6> from which
     * TaskSpace can be constructed.
    */
    const std::map<TaskSpaceTangentDimension, bool> stdMap{
        {TaskSpaceTangentDimension::Vx, true},
        {TaskSpaceTangentDimension::Vy, false},
        {TaskSpaceTangentDimension::Vz, true},
        {TaskSpaceTangentDimension::Wx, true},
        {TaskSpaceTangentDimension::Wy, false},
        {TaskSpaceTangentDimension::Wz, false}};
    const std::array<bool, 6> stdArrayBool({true, false, true, true, false, false});

    /**
     * Constructors
    */

    /**
     * Default constructor.
     * Creates an object with
     * {
     *     "Vx": true,
     *     "Vy": true,
     *     "Vz": true,
     *     "Wx": true,
     *     "Wy": true,
     *     "Wz": true
     * }
     * map.
    */
    TaskSpace taskSpace0;

    /**
     * Constructors from std::map<TaskSpaceTangentDimension, bool>, std::array<bool, 6>
     * and initialiser_list
    */
    TaskSpace taskSpace1(stdMap);
    TaskSpace taskSpace2(stdArrayBool);
    TaskSpace taskSpace3({true, false, true, true, false, false});

    /**
     * Assignment operators from std::map<TaskSpaceTangentDimension, bool>, std::array<bool, 6>
     * and initialiser_list
    */
    taskSpace1 = stdMap;
    taskSpace1 = stdArrayBool;
    taskSpace1 = {true, false, true, true, false, false};

    /**
     * Dimension operator, returns the number of dimensions in the TaskSpace
    */
    taskSpace1.dimension();

    /**
     * Linear dimension operator, returns the number of linear dimensions in the TaskSpace
    */
    taskSpace1.linearDimension();

    /**
     * Angular dimension operator, returns the number of angular dimensions in the TaskSpace
    */
    taskSpace1.angularDimension();

    /**
     * Function for aquiring a representation of the task space.
     * Returns a modyfied identity matrix with rows corresponding to unused dimensions
     * missing.
     */
    taskSpace1.getNoRowsMatrix();

    /**
     * Function for aquiring a representation of the task space.
     * Returns a modyfied identity matrix with rows corresponding to unused dimensions
     * filled with zeros.
     */
    taskSpace1.getZeroRowsMatrix();

    /**
     * Function for aquiring a representation of the task space.
     * Returns a modyfied matrix with rows corresponding to unused dimensions
     * filled with NaNs.
     */
    taskSpace1.getNaNRowsMatrix();

    /**
     * Function for aquiring a representation of the task space.
     * Returns an eigen vector with rows corresponding to unused dimensions missing.
     */
    taskSpace1.getVector6d();

    /**
     * Size function, returns number of possible coordinates (in the task space or not).
     * Always equal to 6.
     */
    taskSpace1.size();

    /**
     * Index operator, indexed by 'TaskSpaceTangentDimension' enum.
     */
    taskSpace1[TaskSpaceTangentDimension::Wx] = inputBool;
    outputBool = taskSpace1[TaskSpaceTangentDimension::Wx];

    /**
     * Index operator, indexed by [0, 6].
     */
    taskSpace1[4] = inputBool;
    outputBool = taskSpace1[4];

    /**
     * Print to stream operator.
    */
    std::cout << taskSpace1 << std::endl;

    return 0;
}
