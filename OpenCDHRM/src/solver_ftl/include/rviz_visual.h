/**
 * @file rviz_visual.h
 * @author Mingrui Luo (luomingrui2020@ia.ac.cn)
 * @brief RVIZ based visualization library
 * @version 0.1
 */

#ifndef RVIZ_VISUAL_H
#define RVIZ_VISUAL_H

#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Header.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>

/**
 * @brief Spherical obstacle
 * 
 */
struct SphObs
{
  double r;   /// Radius
  Vector3d p; /// Center position
};

/**
 * @brief Convert to JointState message
 * 
 * @param jsd Joint values
 * @param ang_rot Base rotation angle
 * @return sensor_msgs::JointState 
 */
sensor_msgs::JointState ShowModel(vector<vector<double>> jsd, double ang_rot)
{
  sensor_msgs::JointState js;
  js.header = std_msgs::Header();
  js.header.stamp = ros::Time::now();
  vector<double> jlist;
  vector<string> jname;
  jlist.push_back(ang_rot / 180 * M_PI);
  jname.push_back("joint_rot");
  jlist.push_back(jsd[0][1] / 180 * M_PI);
  jname.push_back("joint_1");
  for (size_t i = 1; i < jsd.size(); i++)
  {
    jlist.push_back(jsd[i][0] / 180 * M_PI);
    jlist.push_back(jsd[i][1] / 180 * M_PI);
    jname.push_back("joint_" + to_string(i * 2));
    jname.push_back("joint_" + to_string(i * 2 + 1));
  }
  js.position = jlist;
  js.name = jname;
  return js;
}

/**
 * @brief Convert to JointState message
 * 
 * @param jlist JointState Values
 * @return sensor_msgs::JointState 
 */
sensor_msgs::JointState ShowModel(vector<double> jlist)
{
  sensor_msgs::JointState js;
  js.header = std_msgs::Header();
  js.header.stamp = ros::Time::now();
  vector<string> jname;
  jname.push_back("joint_rot");
  for (size_t i = 1; i < jlist.size(); i++)
  {
    jname.push_back("joint_" + to_string(i));
  }
  js.position = jlist;
  js.name = jname;
  return js;
}

/**
 * @brief Draw obstacle points
 * 
 * @return visualization_msgs::MarkerArray 
 */
visualization_msgs::MarkerArray Plotobs(vector<SphObs> obslist)
{
  visualization_msgs::MarkerArray markerlist;
  for (size_t i = 0; i < obslist.size(); i++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "obs";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = obslist[i].p[0] / 1000.0;
    marker.pose.position.y = obslist[i].p[1] / 1000.0;
    marker.pose.position.z = obslist[i].p[2] / 1000.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = obslist[i].r / 1000.0;
    marker.scale.y = obslist[i].r / 1000.0;
    marker.scale.z = obslist[i].r / 1000.0;
    marker.color.a = 0.6;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.2;
    markerlist.markers.push_back(marker);
  }
  return markerlist;
}

/**
 * @brief Draw position points
 * 
 * @param points position points
 * @param type Visualization Type
 * @param r Red Color
 * @param g Green Color
 * @param b Blue Color
 * @param a Transparency
 * @param w Line Width
 * @param id ID
 * @return visualization_msgs::Marker 
 */
visualization_msgs::Marker PlotPoints(vector<Vector3d> points, uint8_t type = 4,
                                      float r = 1.0, float g = 0.0, float b = 0.0, float a = 1.0, float w = 0.01, uint64_t id = 0)
{
  visualization_msgs::Marker points_list;
  points_list.header.frame_id = "base_link";
  points_list.header.stamp = ros::Time::now();
  points_list.ns = "points_" + to_string(id);
  points_list.action = visualization_msgs::Marker::ADD;
  points_list.lifetime = ros::Duration(10);
  points_list.pose.orientation.w = 1.0;
  if (id == 0)
    points_list.id = (uint64_t)ros::Time::now().toNSec();
  else
    points_list.id = id;
  points_list.type = type;

  if (type == 0)
    points_list.scale.x = w * 0.4;
  else
    points_list.scale.x = w;
  points_list.scale.y = w;
  points_list.scale.z = w;
  points_list.color.r = r;
  points_list.color.g = g;
  points_list.color.b = b;
  points_list.color.a = a;

  for (size_t i = 0; i < points.size(); i++)
  {
    geometry_msgs::Point p;
    p.x = points[i][0] / 1000.0;
    p.y = points[i][1] / 1000.0;
    p.z = points[i][2] / 1000.0;
    points_list.points.push_back(p);
  }
  return points_list;
}

/**
 * @brief Draw Text
 * 
 * @param text Text
 * @param x Position X
 * @param y Position Y
 * @param z Position Z
 * @return visualization_msgs::Marker 
 */
visualization_msgs::Marker PlotText(string text, double x, double y, double z,
                                    float r = 0.0, float g = 0.0, float b = 0.0, float a = 1.0, float w = 0.1, uint64_t id = 0)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "text_" + to_string(id);
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.id = id;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

  marker.scale.z = w;
  marker.color.b = b;
  marker.color.g = g;
  marker.color.r = r;
  marker.color.a = a;

  marker.text = text;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;

  return marker;
}
//

/**
 * @brief Draw STL model
 * 
 * @param path Model Path
 * @param x Model Origin X
 * @param y Model Origin Y
 * @param z Model Origin Z
 * @param r Red Color
 * @param g Green Color
 * @param b Blue Color
 * @param a Transparency
 * @param s Scale
 * @param id ID
 * @return visualization_msgs::Marker 
 */
visualization_msgs::Marker PlotMesh(string path, double x, double y, double z,
                                    float r = 0.5, float g = 0.5, float b = 0.5, float a = 1.0, float s = 1.0, uint64_t id = 0)
{
  visualization_msgs::Marker marker;

  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.mesh_resource = path;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = id;
  marker.lifetime = ros::Duration(0);
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "world";
  marker.ns = "mesh_" + to_string(id);
  //set position
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  //set pose/orientation
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  //set scale
  marker.scale.x = s;
  marker.scale.y = s;
  marker.scale.z = s;
  // set color
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;
  return marker;
}

#endif