/*********************************************************************
*
* Software License Agreement (All Rights Reserved)
*
*  Copyright (c) 2013, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification is not permitted without the explicit permission of 
*  Willow Garage Inc.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Sachin Chitta
*********************************************************************/

#include <moveit/workspace_analysis/workspace_analysis.h>
#include <eigen_conversions/eigen_msg.h>
#include <fstream>
#include <iostream>

namespace moveit_workspace_analysis
{

WorkspaceAnalysis::WorkspaceAnalysis(const planning_scene::PlanningSceneConstPtr &planning_scene,
                                     bool position_only,
                                     double joint_limits_penalty_multiplier): planning_scene_(planning_scene), position_only_ik_(position_only), canceled_(false)
{
  state_validity_callback_fn_ = boost::bind(&WorkspaceAnalysis::isIKSolutionCollisionFree, this, _1, _2);  
  kinematics_metrics_.reset(new kinematics_metrics::KinematicsMetrics(planning_scene->getCurrentState().getRobotModel()));
  kinematics_metrics_->setPenaltyMultiplier(joint_limits_penalty_multiplier);  
}

bool WorkspaceAnalysis::isIKSolutionCollisionFree(robot_state::JointStateGroup *joint_state_group,
                                                  const std::vector<double> &ik_solution)
{
  joint_state_group->setVariableValues(ik_solution);
  bool result = !planning_scene_->isStateColliding(*joint_state_group->getRobotState(), joint_state_group->getName());
  return result;  
}

std::vector<geometry_msgs::Pose> WorkspaceAnalysis::sampleUniform(const moveit_msgs::WorkspaceParameters &workspace, 
                                                                  const std::vector<geometry_msgs::Quaternion> &orientations,
                                                                  double x_resolution,
                                                                  double y_resolution,
                                                                  double z_resolution) const
{
  std::vector<geometry_msgs::Pose> results;
  std::vector<geometry_msgs::Quaternion> rotations = orientations;  
  if(orientations.empty())
  {
    geometry_msgs::Quaternion quaternion;
    quaternion.w = 1.0;    
    rotations.push_back(quaternion);    
  }
  double x_min = workspace.min_corner.x;
  double y_min = workspace.min_corner.y;
  double z_min = workspace.min_corner.z;

  unsigned int x_num_points,y_num_points,z_num_points;
  double x_dim = std::fabs(workspace.min_corner.x - workspace.max_corner.x);
  double y_dim = std::fabs(workspace.min_corner.y - workspace.max_corner.y);
  double z_dim = std::fabs(workspace.min_corner.z - workspace.max_corner.z);

  x_num_points = (unsigned int) (x_dim/x_resolution) + 1;
  y_num_points = (unsigned int) (y_dim/y_resolution) + 1;
  z_num_points = (unsigned int) (z_dim/z_resolution) + 1;

  ROS_DEBUG("Cache dimension (num grid points) in (x,y,z): %d %d %d",x_num_points,y_num_points,z_num_points);
  geometry_msgs::Pose pose;  
  for(std::size_t i=0; i < x_num_points; ++i)
  {
    pose.position.x = x_min + i * x_resolution;
    for(std::size_t j=0; j < y_num_points; ++j)
    {
      pose.position.y = y_min + j * y_resolution;
      for(std::size_t k=0; k < z_num_points; ++k)
      {
        pose.position.z = z_min + k * z_resolution;
        for(std::size_t m=0; m < rotations.size(); ++m)
        {
          pose.orientation = rotations[m];
          results.push_back(pose);          
        }
      }
    }
  }
  ROS_INFO("Generated %d samples for workspace points",(int) results.size());
  return results;
}

WorkspaceMetrics WorkspaceAnalysis::computeMetrics(const moveit_msgs::WorkspaceParameters &workspace,
                                                   const std::vector<geometry_msgs::Quaternion> &orientations,
                                                   robot_state::JointStateGroup *joint_state_group,
                                                   double x_resolution,
                                                   double y_resolution,
                                                   double z_resolution) const
{
  WorkspaceMetrics metrics;
  if(!joint_state_group || !planning_scene_)
  {
    ROS_ERROR("Joint state group and planning scene should not be null");
    return metrics;
  } 
  std::vector<geometry_msgs::Pose> points = sampleUniform(workspace, orientations, x_resolution, y_resolution, z_resolution);
  metrics.group_name_ = joint_state_group->getName();
  metrics.robot_name_ = joint_state_group->getRobotState()->getRobotModel()->getName();
  metrics.frame_id_ =  joint_state_group->getRobotState()->getRobotModel()->getModelFrame();  
  
  for(std::size_t i=0; i < points.size(); ++i)
  {   
    if(!ros::ok() || canceled_)
      return metrics;    
    bool found_ik = joint_state_group->setFromIK(points[i], 1, 0.01, state_validity_callback_fn_);
    if(found_ik)
    {
      ROS_DEBUG("Found IK: %d", (int) i);      
      metrics.points_.push_back(points[i]);      
      updateMetrics(joint_state_group, metrics);      
    }
  }
  return metrics;  
}

WorkspaceMetrics WorkspaceAnalysis::computeMetricsFK(robot_state::JointStateGroup *joint_state_group,
                                                     unsigned int max_attempts,
                                                     const ros::WallDuration &max_duration,
                                                     const std::map<std::string, std::vector<double> > &fixed_joint_values) const
{
  ros::WallTime start_time = ros::WallTime::now();  
  WorkspaceMetrics metrics;
  if(!joint_state_group || !planning_scene_)
  {
    ROS_ERROR("Joint state group and planning scene should not be null");
    return metrics;
  } 
  if(!fixed_joint_values.empty())
  {
    for(std::map<std::string, std::vector<double> >::const_iterator iter=fixed_joint_values.begin(); iter != fixed_joint_values.end(); ++iter)
    {
      if(!joint_state_group->hasJointState((*iter).first))
      {
        ROS_ERROR("Could not find joint: %s in joint group: %s", (*iter).first.c_str(), joint_state_group->getName().c_str());
        return metrics;
      }      
    }  
  }
  metrics.group_name_ = joint_state_group->getName();
  metrics.robot_name_ = joint_state_group->getRobotState()->getRobotModel()->getName();
  metrics.frame_id_ =  joint_state_group->getRobotState()->getRobotModel()->getModelFrame();  
  
  //Find end-effector link
  std::string link_name = joint_state_group->getJointModelGroup()->getLinkModelNames().back();  
  robot_state::LinkState *link_state = joint_state_group->getRobotState()->getLinkState(link_name);  

  for(std::size_t i=0; i < max_attempts; ++i)
  {   
    if(!ros::ok() || canceled_ || (ros::WallTime::now()-start_time) >= max_duration)
      return metrics;    
    joint_state_group->setToRandomValues();
    if(!fixed_joint_values.empty())
    {
      for(std::map<std::string, std::vector<double> >::const_iterator iter=fixed_joint_values.begin(); iter != fixed_joint_values.end(); ++iter)
      {
        joint_state_group->getJointState((*iter).first)->setVariableValues((*iter).second);        
      }      
      joint_state_group->updateLinkTransforms();    
    }
    if(planning_scene_->isStateColliding(*joint_state_group->getRobotState(), joint_state_group->getName()))
      continue;    
    const Eigen::Affine3d &link_pose = link_state->getGlobalLinkTransform();
    geometry_msgs::Pose pose;    
    tf::poseEigenToMsg(link_pose,pose);
    metrics.points_.push_back(pose);    
    updateMetrics(joint_state_group, metrics);      
  }
  return metrics;  
}

void WorkspaceAnalysis::updateMetrics(robot_state::JointStateGroup *joint_state_group,
                                      moveit_workspace_analysis::WorkspaceMetrics &metrics) const
{
  double manipulability_index;      
  kinematics_metrics_->getManipulabilityIndex(*joint_state_group->getRobotState(), 
                                              joint_state_group->getJointModelGroup(),
                                              manipulability_index,
                                              position_only_ik_);
  std::pair<double,int> distance = joint_state_group->getMinDistanceToBounds();      
  std::vector<double> joint_values;      
  joint_state_group->getVariableValues(joint_values);
  metrics.joint_values_.push_back(joint_values);      
  metrics.manipulability_.push_back(manipulability_index);      
  metrics.min_distance_joint_limits_.push_back(distance.first);      
  metrics.min_distance_joint_limit_index_.push_back(distance.second);      
}

bool WorkspaceMetrics::writeToFile(const std::string &filename, const std::string &delimiter, bool exclude_strings)
{
  ROS_DEBUG("Writing %d total points to file: %s",(int) points_.size(),filename.c_str());  
  if(points_.size() != manipulability_.size() || points_.size() != joint_values_.size() || points_.size() != min_distance_joint_limits_.size())
  {
    ROS_ERROR("Workspace metrics not fully formed");
    return false;
  }  
  
  std::ofstream file;
  file.open(filename.c_str());
  if(!file.is_open())
  {
    ROS_DEBUG("Could not open file: %s",filename.c_str());
    return false;    
  }
  if(file.good())
  {
    if(!exclude_strings)
    {
      file << robot_name_ << std::endl;
      file << group_name_ << std::endl;
      file << frame_id_ << std::endl;
    }    
    for(std::size_t i=0; i < points_.size(); ++i)
    {
      if(joint_values_[i].empty())
        continue;      
      file << points_[i].position.x << delimiter << points_[i].position.y << delimiter << points_[i].position.z << delimiter;
      file << points_[i].orientation.x << delimiter << points_[i].orientation.y  << delimiter << points_[i].orientation.z << delimiter << points_[i].orientation.w << delimiter;
      for(std::size_t j=0; j < joint_values_[i].size(); ++j)
        file << joint_values_[i][j] << delimiter;        
      file << manipulability_[i] << delimiter << min_distance_joint_limits_[i] << delimiter << min_distance_joint_limit_index_[i] << std::endl;
      //file << manipulability_[i] << delimiter << min_distance_joint_limits_[i] << std::endl;
    }
  }  
  file.close();
  return true;   
}

bool WorkspaceMetrics::readFromFile(const std::string &filename, unsigned int num_joints)
{
  std::ifstream file;
  file.open(filename.c_str());
  if(!file.is_open())
  {
    ROS_DEBUG("Could not open file: %s",filename.c_str());
    return false;    
  }

  std::vector<double> joint_values(num_joints);
  std::vector<std::string> name_strings;  
  for(std::size_t i=0; i < 3; ++i)
  {    
    std::string name_string;
    std::getline(file, name_string);
    name_strings.push_back(name_string);    
  }  

  robot_name_ = name_strings[0];
  group_name_ = name_strings[1];
  frame_id_ = name_strings[2];  

  while(!file.eof() && file.good())
  {
    std::string line;
    std::getline(file, line);

    std::stringstream line_stream(line);
    std::string field;
    std::vector<double> record;    
    while( std::getline(line_stream, field, ','))
    {
      double f(0.0);
      std::stringstream field_stream(field);
      field_stream >> f;
      record.push_back(f);
    }    
    if(record.empty())
      continue;    
    ROS_DEBUG("Read: %d records", (int) record.size());    
    geometry_msgs::Pose pose;
    pose.position.x = record[0];
    pose.position.y = record[1];
    pose.position.z = record[2];
    pose.orientation.x = record[3];
    pose.orientation.y = record[4];
    pose.orientation.z = record[5];
    pose.orientation.w = record[6];    
    points_.push_back(pose);

    for(std::size_t i = 0; i < num_joints; ++i)
      joint_values[i] = record[7+i];
    joint_values_.push_back(joint_values);
    
    manipulability_.push_back(record[7+num_joints]);
    min_distance_joint_limits_.push_back(record[8+num_joints]);
    min_distance_joint_limit_index_.push_back(record[9+num_joints]);    
  }      
  ROS_DEBUG("Done reading");  
  file.close();
  return true;   
}

visualization_msgs::Marker WorkspaceMetrics::getMarker(double marker_scale, unsigned int id, const std::string &ns) const
{
  visualization_msgs::Marker marker;  
  marker.type = marker.SPHERE_LIST;
  marker.action = 0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = marker_scale;
  marker.scale.y = marker_scale;
  marker.scale.z = marker_scale;
  marker.id = id;
  marker.ns = ns;  

  double max_manip(-1.0), min_manip(std::numeric_limits<double>::max());
  
  for(std::size_t i=0; i < points_.size(); ++i)
  {
    if(manipulability_[i] > max_manip)
      max_manip = manipulability_[i];
    if(manipulability_[i] < min_manip)
      min_manip = manipulability_[i];    
  }
  
  for(std::size_t i = 0; i < points_.size(); ++i)
  {
    marker.points.push_back(points_[i].position);
    std_msgs::ColorRGBA color;
    color.a = 1.0;
    color.g = 0.0;
    color.r = manipulability_[i]/max_manip;
    color.b = 1 - manipulability_[i]/max_manip;      
    marker.colors.push_back(color);      
  }    
  return marker;  
}

}
