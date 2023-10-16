/**
 * @file FTLSolver.cpp
 * @author Mingrui Luo (luomingrui2020@ia.ac.cn)
 * @brief Follow-the-Leader (FTL) Solver Framework
 * @version 0.1
 */

#include "FTLSolver.h"

bool FTLSolver::Follow(double length,bool optnmin,bool reverse)
{
    if (reverse==true)
    {
        switch (method)
        {
            case method_SCP: {length=SCPSolver.CPSeq[SCPSolver.CPSeq.size()-1].length-length;break;}
        }
    }
    bool flag=false;
    switch (method)
    {
        case method_SCP: {flag=SCPSolver.Follow(length,apoints);break;}
    }
    return (flag)&&(CheckLimits());
}

bool FTLSolver::StepMove(double step,double alpha,double beta,bool optnmin)
{
    GenStep(step,alpha,beta);
    return Follow(0,optnmin,true);
}

bool FTLSolver::CheckLimits(void) {return true;}

void FTLSolver::GenStep(double step,double alpha,double beta)
{
    if (step>0)
    {
        Vector3d tip=cpoints[cpoints.size()-1];
        Vector3d vect=cpoints[cpoints.size()-1]-cpoints[cpoints.size()-2];
        Vector3d nvect=vect_rot_by_sph(vect,alpha,beta,step/vect.norm());
        AddCP(tip+nvect);
    }
}

void FTLSolver::SetMBase(vector<Vector3d> bpoints_)
{
    cpoints.clear();
    cpoints.assign(bpoints_.begin(), bpoints_.end());
    N_lk=bpoints_.size();
    switch (method)
    {
        case method_SCP: {SCPSolver.Set_params(N_lk,link_lengths[0]);break;}
    }
}

void FTLSolver::AddCP(vector<Vector3d> cpoints_,size_t offset)
{
    cpoints.erase(cpoints.end()-min(offset,cpoints.size()-N_lk),cpoints.end());
    cpoints.insert(cpoints.end(), cpoints_.begin(), cpoints_.end());

    switch (method)
    {
        case method_SCP: {SCPSolver.Unit_Init(cpoints);break;}
    }
}

void FTLSolver::AddCP(Vector3d cpoint)
{
    cpoints.push_back(cpoint);
    switch (method)
    {
        case method_SCP: {SCPSolver.Unit_Add(cpoint);break;}
    }
}