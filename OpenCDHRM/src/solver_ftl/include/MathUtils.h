/**
 * @file MathUtils.h
 * @author Mingrui Luo (luomingrui2020@ia.ac.cn)
 * @brief Basic Mathematical Functions
 * @version 0.1
 */

#ifndef MATH_UTILS_H
#define MATH_UTILS_H

/// Need Eigen library
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>

using namespace std;
using namespace Eigen;

/**
 * @brief Calculate the angle between vect1 and vect2
 */
double vect_get_ang(Vector3d vect1, Vector3d vect2);
/**
 * @brief Spherical coordinate system to Cartesian coordinate system
 */
Vector3d Sph2Cart(Vector3d p);
/**
 * @brief Cartesian coordinate system to Spherical coordinate system
 */
Vector3d Cart2Sph(Vector3d p);
/**
 * @brief Rotating a vector in a spherical coordinate system
 */
Vector3d vect_rot_by_sph(Vector3d vect, double dalpha, double dbeta, double scal = 1.0);

#endif