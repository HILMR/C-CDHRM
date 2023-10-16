/**
 * @file test.cpp
 * @author Mingrui Luo (luomingrui2020@ia.ac.cn)
 * @brief Demonstration Program for FTL Solver
 * @version 0.1
 */

#include "FTL_CCDHRM.h"
#include "rviz_visual.h"
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>

double fRand(double min, double max)
{
    double f = (double)rand() / RAND_MAX;
    return min + f * (max - min);
}

vector<vector<double>> readdata(string input)
{
    vector<vector<double>> data;
    stringstream ss(input);
    vector<string> substrings;
    string item;

    while (getline(ss, item, ';'))
    {
        stringstream ss2(item);
        vector<double> subdata;
        while (getline(ss2, item, ','))
        {
            subdata.push_back(stod(item));
        }
        data.push_back(subdata);
    }
    return data;
}

void LoadPlanning(string path, vector<SphObs> &obslist, vector<Vector3d> &cpoints)
{
    ifstream file_results(path);
    string line;
    vector<vector<double>> data;

    /// The first line of the data file is obstacle information
    getline(file_results, line);
    data = readdata(line);
    for (size_t i = 0; i < data.size(); i++)
    {
        SphObs obs;
        obs.p = Vector3d(data[i][0], data[i][1], data[i][2]);
        obs.r = data[i][3];
        obslist.push_back(obs);
    }

    /// The second line of the data file is the control point
    getline(file_results, line);
    data = readdata(line);
    for (size_t i = 0; i < data.size(); i++)
    {
        cpoints.push_back(Vector3d(data[i][0], data[i][1], data[i][2]));
    }

    file_results.close();
}

bool Processing(int &flag, double &length, double &lengthmax, FTL_CCDHRM &FTLSolver,
                sensor_msgs::JointState &js, visualization_msgs::MarkerArray &lines)
{
    vector<vector<double>> jsl;
    double ang_rot;
    /// Stepwise following mode
    if (flag == 0)
    {
        length = FTLSolver.SCPSolver.CPSeq[FTLSolver.SCPSolver.CPSeq.size() - 1].length + 20;
        if (length < 2000)
        {
            FTLSolver.StepMove(20, fRand(-10, 10), fRand(-10, 10), jsl, ang_rot, true);
        }
        else
        {
            flag = 1;
            lengthmax = FTLSolver.SCPSolver.CPSeq[FTLSolver.SCPSolver.CPSeq.size() - 1].length;
        }
    }

    /// Loop following mode
    if ((flag == 1) || (flag == 2))
    {
        if (flag == 1)
        {
            length = length + 10;
            if (length > lengthmax)
            {
                length = lengthmax;
                flag = 2;
            }
        }
        else
        {
            length = length - 10;
            if (length < 0)
            {
                length = 0;
                flag = 1;
            }
        }
        FTLSolver.Follow(length, jsl, ang_rot, true);
    }

    /// Visualization results
    cout << "length=" << length << endl;
    js = ShowModel(jsl, ang_rot);

    vector<Vector3d> tippos;
    lines.markers.push_back(PlotPoints(FTLSolver.apoints, 4, 0.5, 1.0, 0.8, 0.8, 0.01));
    lines.markers.push_back(PlotPoints(FTLSolver.apoints, 7, 1.0, 0.5, 0.2, 0.8, 0.015));
    tippos.clear();
    tippos.push_back(FTLSolver.apoints[12] - 0.2 * (FTLSolver.apoints[12] - FTLSolver.apoints[11]));
    tippos.push_back(FTLSolver.apoints[12]);
    lines.markers.push_back(PlotPoints(tippos, 0, 0.2, 0.6, 1.0, 1.0, 0.02));
    tippos.clear();
    tippos.push_back(FTLSolver.apoints[0]);
    lines.markers.push_back(PlotPoints(tippos, 7, 1.0, 0.5, 0.2, 1.0, 0.03));
}

int main(int argc, char **argv)
{
    int sed = time(0);
    cout << "Random Seed=" << sed << endl;
    srand(sed);

    ros::init(argc, argv, "FTLSolver_CCDHRM");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1000);
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 0);
    sensor_msgs::JointState js;
    visualization_msgs::MarkerArray lines;

    /// Establish an FTL solver
    FTL_CCDHRM FTLSolver(vector<double>(12, 200), 420,
                         atan2(0.40808, -0.098751) / M_PI * 180.0, Vector3d(0, -92, 557), 13);
    vector<SphObs> obslist;
    vector<Vector3d> cpoints;

    double length = 0;
    double lengthmax;
    int flag = 0;

    ros::Rate loop_rate(20);
    if ((argc > 1) && (string(argv[1]) == "load"))
    {
        /// Read pre calculated planning results
        LoadPlanning(string(argv[2]), obslist, cpoints);
        flag = 1;
    }
    else
    {
        /// Stepwise random generation path
        cpoints.push_back(Vector3d(0, -92, 557 + 420));
        cpoints.push_back(Vector3d(300, -92, 557 + 420));
    }

    /// Initialize control points
    FTLSolver.InitCP(cpoints);
    lengthmax = FTLSolver.SCPSolver.CPSeq[FTLSolver.SCPSolver.CPSeq.size() - 1].length;

    while (ros::ok())
    {
        if (obslist.size() > 0)
        {
            lines = Plotobs(obslist);
        }
        else
        {
            lines.markers.clear();
        }

        /// Start solving
        Processing(flag, length, lengthmax, FTLSolver, js, lines);

        pub.publish(js);
        vis_pub.publish(lines);
        loop_rate.sleep();
    }
}