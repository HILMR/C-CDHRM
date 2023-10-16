/**
 * @file FTL_CCDHRM.cpp
 * @author Mingrui Luo (luomingrui2020@ia.ac.cn)
 * @brief Follow-the-Leader (FTL) Solver of C-CDHRMs
 * @version 0.1
 */

#include "FTL_CCDHRM.h"

void FTL_CCDHRM::Points2Joints(vector<vector<double>> &jointvalues, double &ang_rot, bool optnmin)
{
    if (optnmin)
        CompletePoints();
    jointvalues.clear();
    Vector3d bpoint = apoints[0] - base;
    double beta1 = vect_get_ang(base - apoints[0], apoints[1] - apoints[0]);
    double rot_base = FixAngles(bpoint, beta1, ang_rot);
    if (rot_xoff_ena)
    {
        Points2JointValues(apoints, jointvalues, Vector3d(0, rot_base, 0), first_rot_z, cross_rot, rot_xoff);
    }
    else
    {
        Points2JointValues(apoints, jointvalues, Vector3d(0, rot_base, 0), first_rot_z, cross_rot);
    }
    jointvalues[0][1] = beta1;
}

bool FTL_CCDHRM::Follow(double length, vector<vector<double>> &jointvalues, double &ang_rot, bool optnmin, bool reverse)
{
    bool flag = FTLSolver::Follow(length, optnmin, reverse);
    if (flag == false)
        return flag;
    Points2Joints(jointvalues, ang_rot, optnmin);
    return flag;
}

bool FTL_CCDHRM::StepMove(double step, double alpha, double beta, vector<vector<double>> &jointvalues, double &ang_rot, bool optnmin)
{
    if (step < 0)
        return false;
    bool flag = FTLSolver::StepMove(step, alpha, beta, optnmin);
    if (flag == false)
        return flag;
    Points2Joints(jointvalues, ang_rot, optnmin);
    return flag;
}

vector<Vector3d> FTL_CCDHRM::GenBPoints(void)
{
    vector<Vector3d> bpoints;
    double theta;
    for (size_t i = 0; i < N_lk; i++)
    {
        if (i == 0)
            theta = base_rot_ang;
        else
        {
            double dtheta = asin(link_lengths[i - 1] / r_base / 2);
            theta = theta + 2 * dtheta;
        }
        bpoints.push_back(base + r_base * Vector3d(cos(theta), 0, sin(theta)));
    }
    reverse(bpoints.begin(), bpoints.end());
    return bpoints;
}

void FTL_CCDHRM::InitCP(vector<Vector3d> cpoints_)
{
    vector<Vector3d> cpoints_in;
    cpoints_in.assign(cpoints_.begin() + 1, cpoints_.end());
    Vector3d btarget = cpoints_[0] - base;
    base_rot_ang = atan2(btarget[2], btarget[0]);
    SetMBase(GenBPoints());
    AddCP(cpoints_in);
}

void FTL_CCDHRM::CompletePoints(void)
{
    CalInActivePoints(apoints, link_lengths);
}

void FTL_CCDHRM::SetRotX(vector<double> rot_xoff_)
{
    rot_xoff_ena = true;
    rot_xoff = rot_xoff_;
}