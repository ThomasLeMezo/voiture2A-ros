//
// Created by lemezoth on 01/01/24.
//

#include "Fusion.h"
#include <vector>


//------------------------------------------------------------------------------
// std::vector conversion

/**
 * @brief Converts a std::vector to a FusionVector.
 * @param vector Vector.
 * @return FusionVector.
 */
static inline FusionVector FusionVectorFromStdVector(const std::vector<double> &vector) {
    const FusionVector result = {.axis = {
            .x = static_cast<float>(vector[0]),
            .y = static_cast<float>(vector[1]),
            .z = static_cast<float>(vector[2]),
    }};
    return result;
}

/**
 * @brief Converts a std::vector to a FusionQuaternion.
 * @param matrix
 * @return
 */
static inline FusionMatrix FusionMatrixFromStdVector(const std::vector<double> &matrix) {
    const FusionMatrix result = {.element = {
            .xx = static_cast<float>(matrix[0]),
            .xy = static_cast<float>(matrix[1]),
            .xz = static_cast<float>(matrix[2]),
            .yx = static_cast<float>(matrix[3]),
            .yy = static_cast<float>(matrix[4]),
            .yz = static_cast<float>(matrix[5]),
            .zx = static_cast<float>(matrix[6]),
            .zy = static_cast<float>(matrix[7]),
            .zz = static_cast<float>(matrix[8]),
    }};
    return result;
}

/**
 * @brief Converts a std::vector to a FusionQuaternion.
 * @param quaternion
 * @return
 */
static inline FusionQuaternion FusionQuaternionFromStdVector(const std::vector<double> &quaternion) {
    const FusionQuaternion result = {.element = {
            .w = static_cast<float>(quaternion[0]),
            .x = static_cast<float>(quaternion[1]),
            .y = static_cast<float>(quaternion[2]),
            .z = static_cast<float>(quaternion[3]),
    }};
    return result;
}

/**
 * @brief Converts a std::vector to a FusionEuler.
 * @param euler
 * @return
 */
static inline FusionEuler FusionEulerFromStdVector(const std::vector<double> &euler) {
    const FusionEuler result = {.angle = {
            .roll = static_cast<float>(euler[0]),
            .pitch = static_cast<float>(euler[1]),
            .yaw = static_cast<float>(euler[2]),
    }};
    return result;
}

/**
 * @brief Converts a FusionVector to a std::vector.
 * @param vector Vector.
 * @return std::vector.
 */
static inline std::vector<double> FusionVectorToStdVector(const FusionVector &vector) {
    return {vector.axis.x, vector.axis.y, vector.axis.z};
}

/**
 * @brief Converts a FusionMatrix to a std::vector.
 * @param matrix
 * @return
 */
static inline std::vector<double> FusionMatrixToStdVector(const FusionMatrix &matrix) {
    return {matrix.element.xx, matrix.element.xy, matrix.element.xz,
            matrix.element.yx, matrix.element.yy, matrix.element.yz,
            matrix.element.zx, matrix.element.zy, matrix.element.zz};
}

/**
 * @brief Converts a FusionQuaternion to a std::vector.
 * @param quaternion
 * @return
 */
static inline std::vector<double> FusionQuaternionToStdVector(const FusionQuaternion &quaternion) {
    return {quaternion.element.w, quaternion.element.x, quaternion.element.y, quaternion.element.z};
}

/**
 * @brief Converts a FusionEuler to a std::vector.
 * @param euler
 * @return
 */
static inline std::vector<double> FusionEulerToStdVector(const FusionEuler &euler) {
    return {euler.angle.roll, euler.angle.pitch, euler.angle.yaw};
}