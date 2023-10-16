/**
 * @file KM_CCDHRM.h
 * @author Mingrui Luo (luomingrui2020@ia.ac.cn)
 * @brief Kinematics model (KM) of Coiled Hyper-Redundant Cable-Driven Manipulators (C-CDHRM)
 * @version 0.1
 */

#ifndef KM_CCDHRM_H
#define KM_CCDHRM_H

#include "KM_CDHRM.h"

/**
 * @brief Kinematics model (KM) of Coiled Hyper-Redundant Cable-Driven Manipulators (C-CDHRM)
 * 
 */
class KM_CCDHRM : public KM_CDHRM
{
public:
    /**
     * @brief Construct a new C-CDHRM model
     * 
     * @param r_base_ Rotating base radius
     * @param rot_offset_ Rotation installation angle offset (Degree)
     * @param base_ Center position of rotating base
     */
    KM_CCDHRM(double r_base_, double rot_offset_, Vector3d base_) : r_base(r_base_), rot_offset(rot_offset_), base(base_),
                                                                    KM_CDHRM(true, false)
    {
    }

    /**
     * @brief Correct base and root angles
     */
    double FixAngles(Vector3d bpoint, double &beta1, double &ang_rot);
    /**
     * @brief Reverse calculate the original base and root angles
     */
    double FixAngles_inv(Vector3d &bpoint, double &beta1, double &ang_rot);
    /**
     * @brief Find the base point that meet constraints
     */
    bool findbasepoint(double px, double py, double pz, double bx, double by,
                       double bz, double L, double r, double &pbx1, double &pbx2, double &pbz1, double &pbz2);
    /**
     * @brief Calculate inactive position points on the base
     */
    void CalInActivePoints(vector<Vector3d> &points, vector<double> link_lengths);
    /**
     * @brief Limit the rotation angle range of the base
     */
    double LimitBase(double ang_rot, bool reverse, double offset = 10.0);

    double r_base;
    double rot_offset;
    Vector3d base;
};

#endif