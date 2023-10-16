/**
 * @file KM_CDHRM.h
 * @author Mingrui Luo (luomingrui2020@ia.ac.cn)
 * @brief Kinematics model (KM) of Hyper-Redundant Cable-Driven Manipulators (CDHRM)
 * @version 0.1
 */

#ifndef KM_CDHRM_H
#define KM_CDHRM_H

#include "MathUtils.h"

/**
 * @brief Kinematics model (KM) of Hyper-Redundant Cable-Driven Manipulators (CDHRM)
 * 
 */
class KM_CDHRM
{
public:
    /**
     * @brief Construct a new CDHRM model
     * 
     * @param first_rot_z_ Rotate order: true:Z-Y false:Y-Z
     * @param cross_rot_ Assemble order: false:Z-Y-Z-Y-... true:Z-Y-Y-Z-Z-Y-...
     */
    KM_CDHRM(bool first_rot_z_ = true, bool cross_rot_ = false) : first_rot_z(first_rot_z_), cross_rot(cross_rot_)
    {
    }

    /**
     * @brief Position points to Link vectors (Working space->Planing Space)
     */
    vector<Vector3d> Points2Vects(vector<Vector3d> points, bool inverse);
    /**
     * @brief Link vectors to Position points (Planing Space->Working space)
     */
    vector<Vector3d> Vects2Points(vector<Vector3d> vects, Vector3d base, bool inverse);
    /**
     * @brief Position points to Joint values (Planing Space->Configure space)
     */
    vector<Vector3d> Points2JointValues(vector<Vector3d> points, vector<vector<double>> &jointvalues,
                                        Vector3d rot_base = Vector3d(0, 0, 0),
                                        bool first_rot_z = true, bool cross_rot = false,
                                        vector<double> rot_xoffset = vector<double>());
    /**
     * @brief Joint values to Position points (Configure space->Planing Space)
     */
    vector<Vector3d> JointValues2Points(Vector3d base, vector<vector<double>> jointvalues,
                                        vector<double> link_lengths,
                                        Vector3d rot_base = Vector3d(0, 0, 0),
                                        bool first_rot_z = true, bool cross_rot = false,
                                        vector<double> rot_xoffset = vector<double>());

    bool first_rot_z, cross_rot;
};

#endif