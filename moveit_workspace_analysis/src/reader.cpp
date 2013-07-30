/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
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
#include <visualization_msgs/Marker.h>

int main(int argc, char **argv)
{
  ros::init (argc, argv, "workspace_analysis_reader");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  /*Get some ROS params */
  ros::NodeHandle node_handle("~");  
  ros::NodeHandle root_handle;  

  std::string filename;
  if (!node_handle.getParam("filename", filename))
    ROS_FATAL("No filename to read from");  

  moveit_workspace_analysis::WorkspaceMetrics metrics;

  ROS_INFO("Reading from file: %s", filename.c_str());  
 
  if(!filename.empty())
    if(!metrics.readFromFile(filename, 4))
      ROS_INFO("Could not read from file");
    else
      ROS_INFO("Read file");
  

  /*  std::string filename_new = filename+"1";  
  if(!filename_new.empty())
    if(!metrics.writeToFile(filename_new))
      ROS_INFO("Could not write to file");
  */
  ros::Publisher display_publisher = root_handle.advertise<visualization_msgs::Marker>("reachable", 1, true);  
  ros::Publisher display_publisher_2 = root_handle.advertise<visualization_msgs::Marker>("unreachable", 1, true);  
  visualization_msgs::Marker marker_reachable = metrics.getMarker(0.25, 0, "");
  visualization_msgs::Marker marker_unreachable = metrics.getMarker(0.25, 0, "", true);
  marker_reachable.header.frame_id = metrics.frame_id_;  
  marker_reachable.header.stamp = ros::Time::now();  
  marker_unreachable.header.frame_id = metrics.frame_id_;  
  marker_unreachable.header.stamp = ros::Time::now();  

  display_publisher.publish(marker_reachable);
  display_publisher_2.publish(marker_unreachable);
  sleep(20.0);  
  
  ros::shutdown();
  return 0;  
}
