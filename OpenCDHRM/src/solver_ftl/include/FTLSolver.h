/**
 * @file FTLSolver.h
 * @author Mingrui Luo (luomingrui2020@ia.ac.cn)
 * @brief Follow-the-Leader (FTL) Solver Framework
 * @version 0.1
 */

#ifndef FTLSolver_H
#define FTLSolver_H

/// Import Algorithm Library
# include "ALG_SCP.h"

/**
 * @brief Follow-the-Leader (FTL) Solver Framework
 * 
 */
class FTLSolver
{
    public:
    /**
     * @brief Available solving algorithms
     * @note method_SCP: SCP-FTL Algorithm
     * 
     */
    enum Solver_Method {
        method_SCP = 1,
    };

    /**
     * @brief Construct a new FTL Solver
     * 
     * @param link_lengths_ Link lengths
     * @param method_ Solving algorithm
     */
    FTLSolver(vector<double> link_lengths_,Solver_Method method_=method_SCP):
    method(method_),link_lengths(link_lengths_){}

    /******************** Top Level Interface ********************/

    /**
     * @brief Follow a set of initialized path control points
     * 
     * @param length The length of the path currently followed
     * @param optnmin Base optimization
     * @param reverse Path reverse: true length=length_max-length
     * @return true Success
     * @return false Failed
     */
    bool Follow(double length,bool optnmin=true,bool reverse=false);

    /**
     * @brief Incremental generation of control points and stepwise following
     * 
     * @param step Step distance (Nonnegative)
     * @param alpha Orthogonal joint angle around the Z-axis (Degree)
     * @param beta Orthogonal joint angle around the Y-axis (Degree)
     * @param optnmin Base optimization
     * @return true Success
     * @return false Failed
     */
    bool StepMove(double step,double alpha,double beta,bool optnmin=true);

    /******************** Helper Functions ********************/

    /**
     * @brief Constraint check
     * 
     * @return true Success
     * @return false Failed
     */
    virtual bool CheckLimits(void);

    /**
     * @brief Generate step control points
     * 
     * @param step Step distance (Nonnegative)
     * @param alpha Orthogonal joint angle around the Z-axis (Degree)
     * @param beta Orthogonal joint angle around the Y-axis (Degree)
     */
    void GenStep(double step,double alpha=0.0,double beta=0.0);

    /**
     * @brief Set base control points
     * 
     * @param bpoints_ Base control points
     */
    void SetMBase(vector<Vector3d> bpoints_);

    /**
     * @brief Add a set of control points
     * 
     * @param cpoints_ Control points
     * @param offset Insert offset
     */
    void AddCP(vector<Vector3d> cpoints_,size_t offset=0);

    /**
     * @brief Add a control point
     * 
     * @param cpoint A control point
     */
    void AddCP(Vector3d cpoint);

    /******************** Solver Definition ********************/

    ALG_SCP SCPSolver; /// SCP-FTL

    /******************** Global Variables ********************/

    Solver_Method method; /// Solution method
    vector<double> link_lengths; /// Link lengths
    size_t N_lk; /// Number of joint position points
    vector<Vector3d> apoints; /// Actual position points
    vector<Vector3d> cpoints; /// Planning control points
};

#endif