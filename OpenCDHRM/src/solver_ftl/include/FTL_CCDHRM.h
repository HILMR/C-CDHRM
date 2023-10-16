/**
 * @file FTL_CCDHRM.h
 * @author Mingrui Luo (luomingrui2020@ia.ac.cn)
 * @brief Follow-the-Leader (FTL) Solver of C-CDHRMs
 * @version 0.1
 */
#ifndef FTL_CCDHRM_H
#define FTL_CCDHRM_H

#include "FTLSolver.h"
#include "KM_CCDHRM.h"

/**
 * @brief Follow-the-Leader (FTL) Solver of C-CDHRMs
 * 
 */
class FTL_CCDHRM:public FTLSolver, public KM_CCDHRM
{
    public:
    /**
     * @brief Construct a new FTL solver
     * 
     * @param link_lengths_ Link length
     * @param r_base_ Rotating base radius
     * @param rot_offset_ Rotation installation angle offset (Degree)
     * @param base_ Center position of rotating base
     * @param N_lk_ Number of joint position points
     * @param method_ Solving algorithm [method_SCP]
     */
    FTL_CCDHRM(vector<double> link_lengths_,double r_base_,
    double rot_offset_,Vector3d base_,size_t N_lk_,Solver_Method method_=method_SCP):
    FTLSolver(link_lengths_,method_),
    KM_CCDHRM(r_base_,rot_offset_,base_),rot_xoff_ena(false)
    {
        N_lk=N_lk_;
    }
    
    /******************** Top Level Interface ********************/

    /**
     * @brief Position points to Joint values (Planing Space->Configure space)
     * 
     * @param jointvalues Joint values (Degree)
     * @param ang_rot Base rotation angle (Degree)
     * @param optnmin Base optimization: 
     * true: Calculate the active and inactive joints, maintain the solution results for the active joints, and strictly locate the inactive joints on the base
     * false: Without any optimization, it may result in incorrect joint angles
     */
    void Points2Joints(vector<vector<double>> &jointvalues,double &ang_rot,bool optnmin);
    
    /**
     * @brief Follow a set of initialized path control points
     * 
     * @param length The length of the path currently followed
     * @param jointvalues Joint values (Degree)
     * @param ang_rot Base rotation angle (Degree)
     * @param optnmin Base optimization
     * @param reverse Path reverse: true length=length_max-length
     * @return true Success
     * @return false Failed
     */
    bool Follow(double length,vector<vector<double>> &jointvalues,double &ang_rot,bool optnmin=true,bool reverse=false);
    
    /**
     * @brief Incremental generation of control points and stepwise following
     * 
     * @param step Step distance (Nonnegative)
     * @param alpha Orthogonal joint angle around the Z-axis (Degree)
     * @param beta Orthogonal joint angle around the Y-axis (Degree)
     * @param jointvalues Joint values (Degree)
     * @param ang_rot Base rotation angle (Degree)
     * @param optnmin Base optimization
     * @return true Success
     * @return false Failed
     */
    bool StepMove(double step,double alpha,double beta,vector<vector<double>> &jointvalues,double &ang_rot,bool optnmin);
    
    /******************** Helper Functions ********************/

    /**
     * @brief Generate base control points
     * 
     * @return vector<Vector3d> Control points
     */
    vector<Vector3d> GenBPoints(void);

    /**
     * @brief Initialize control points
     * 
     * @param cpoints_ Control points
     */
    void InitCP(vector<Vector3d> cpoints_);

    /**
     * @brief Complete the position points on the base
     * 
     */
    void CompletePoints(void);

    /**
     * @brief Set the X-axis torsion angle
     * 
     * @param rot_xoff_ Torsion correction matrix
     */
    void SetRotX(vector<double> rot_xoff_);

    bool rot_xoff_ena; ///When true, enable X-axis torsion calibration
    vector<double> rot_xoff; ///X-axis torsion calibration parameters
    double base_rot_ang; 
};

#endif