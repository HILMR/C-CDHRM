/**
 * @file ALG_SCP.h
 * @author Mingrui Luo (luomingrui2020@ia.ac.cn)
 * @brief Sliding Control Point (SCP) Algorithm
 * @version 0.1
 */

#ifndef ALG_SCP_H
#define ALG_SCP_H

#include "MathUtils.h"

/**
 * @brief Sliding Control Point (SCP) Algorithm
 * 
 */
class ALG_SCP
{
public:
    ALG_SCP(){};

    struct CPSUnit
    {
        double length;            /// Path Length
        double tmax;              /// Maximum interpolation coefficient
        vector<double> thetas;    /// Control angles
        vector<Vector3d> cpoints; /// Control Points
    };                            /// Control Unit

    /******************** Top Level Interface ********************/

    /**
     * @brief Path following
     * 
     * @param length The length of the path currently followed
     * @param apoints Joint position points
     * @return true Success
     * @return false Failed
     */
    bool Follow(double length, vector<Vector3d> &apoints);

    /******************** Core Functions ********************/

    /**
     * @brief Solver Core
     * 
     * @param t Interpolation coefficient
     * @param apoints Joint position points
     * @param cpoints Control Points
     * @param thetas Control angles
     * @return true Success
     * @return false Failed
     */

    bool Follow_Core(double t, vector<Vector3d> &apoints, vector<Vector3d> cpoints, vector<double> thetas);
    /**
     * @brief Coefficient recursive equation
     * 
     * @param t0 Initial coefficient
     * @param theta Control angle
     * @return double Next coefficient
     */

    double Cal_t(double t0, double theta);

    /**
     * @brief Control angle calculation equation
     * 
     * @param points Control Points
     * @return vector<double> Control angles
     */
    vector<double> Cal_thetas(vector<Vector3d> points);

    /******************** Helper Functions ********************/

    /**
     * @brief Locate control unit
     */
    int Loc_unit(double length, double &t0, size_t &loc);
    /**
     * @brief Processing control units with length=lk_length
     */
    void Unit_EQ(double dis, Vector3d cpoint, vector<Vector3d> &cp_n);
    /**
     * @brief Processing control units with length<lk_length
     */
    void Unit_LE(double dis, Vector3d cpoint, vector<Vector3d> &cp_n);
    /**
     * @brief Processing control units with length>lk_length
     */
    void Unit_GE(double dis, Vector3d cpoint, vector<Vector3d> &cp_n);
    /**
     * @brief Initialize control units
     */
    void Unit_Init(vector<Vector3d> cpoints);
    /**
     * @brief Update a control unit
     */
    void Unit_Update(Vector3d cpoint, vector<Vector3d> &cp_n);
    /**
     * @brief Add a control unit
     */
    void Unit_Add(Vector3d cpoint);
    /**
     * @brief Set parameters
     * 
     * @param N_lk_ Number of joint position points
     * @param lk_length_ Link length
     */
    void Set_params(size_t N_lk_, double lk_length_)
    {
        N_lk = N_lk_;
        lk_length = lk_length_;
    }

    /******************** Global Variables ********************/

    double lk_length;      /// Link length
    size_t N_lk;           /// Number of joint position points
    vector<CPSUnit> CPSeq; /// Control point sequence
};

#endif