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
*********************************************************************/

/* Author: Sachin Chitta, Ioan Sucan */

#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_tools/solid_primitive_dims.h>

#include <object_recognition_msgs/TableArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <moveit_msgs/CollisionObject.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"

#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>

static const std::string ROBOT_DESCRIPTION="robot_description";

class PickPlaceGroup
{
public:
  PickPlaceGroup(moveit::planning_interface::MoveGroup &group, 
		 moveit::planning_interface::PlanningSceneInterface &planning_scene_interface): planning_scene_interface_(planning_scene_interface), 
												group_(group), 
												place_resolution_(0.05), 
												listen_tables_(false), 
												place_left_(true)
  {
    table_dirty_ = false;
    table_subscriber_ = node_handle_.subscribe("table_array", 1, &PickPlaceGroup::tableCallback, this);
    recognition_trigger_publisher_ = node_handle_.advertise<std_msgs::Bool>("/recognize_objects_switch", 20);
    octomap_trigger_publisher_ = node_handle_.advertise<std_msgs::Bool>("/move_group/depth_image_update_wait", 20);
    visualization_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("visualize_place", 20, true);
    collision_object_publisher_ = node_handle_.advertise<moveit_msgs::CollisionObject>("/collision_object", 20);
  }

  bool pick(const std::string &object)
  {
    if(!setupTables())
      return false;
    group_.setSupportSurfaceName("table");
    return group_.pick(object);
  }

  bool triggerDetection()
  {
    std_msgs::Bool msg;
    msg.data = true;
    recognition_trigger_publisher_.publish(msg);
  }
  
  /*
  bool octomapUpdatesOn()
  {
    std_msgs::Bool msg;
    msg.data = false;
    octomap_trigger_publisher_.publish(msg);
  }

  bool octomapUpdatesOff()
  {
    std_msgs::Bool msg;
    msg.data = true;
    octomap_trigger_publisher_.publish(msg);
  }
  */

  void visualize(const std::vector<geometry_msgs::PoseStamped> &poses) const
  {
    ROS_DEBUG("Visualizing: %d place poses", (int) poses.size());
    visualization_msgs::MarkerArray marker;
    for(std::size_t i=0; i < poses.size(); ++i)
    {
      visualization_msgs::Marker m;
      m.action = m.ADD;      
      m.type = m.SPHERE;
      m.ns = "place_locations";
      m.id = i;
      m.pose = poses[i].pose;
      m.header = poses[i].header;

      m.scale.x = 0.02;
      m.scale.y = 0.02;
      m.scale.z = 0.02;

      m.color.r = 1.0;
      m.color.g = 0.0;
      m.color.b = 0.0;
      m.color.a = 1.0;

      marker.markers.push_back(m);      
    }
    visualization_publisher_.publish(marker);
  }

  bool setupTables()
  {
    boost::mutex::scoped_lock tlock(table_lock_);
    if(table_array_.tables.empty())
    {
      ROS_ERROR_STREAM("No tables found");
      return false;
    }
    moveit_msgs::CollisionObject co;

    // Remove the existing table
    co.id = "table";
    co.operation = moveit_msgs::CollisionObject::REMOVE;
    collision_object_publisher_.publish(co);
  
    // Add the new table
    co.operation = moveit_msgs::CollisionObject::ADD;
    co.meshes.push_back(table_array_.tables[0].convex_hull);
    co.mesh_poses.push_back(table_array_.tables[0].pose.pose);
    co.header = table_array_.tables[0].pose.header;    
    collision_object_publisher_.publish(co);
    return true;
  }

  void clear()
  {
    moveit_msgs::CollisionObject co;
    co.operation = moveit_msgs::CollisionObject::REMOVE;
    collision_object_publisher_.publish(co);     
  }

  bool place(const std::string &object, 
	     double height_above_table, 
	     double delta_height=0.01, 
	     unsigned int num_heights=2)
  {
    std::vector<geometry_msgs::PoseStamped> poses;
    ROS_DEBUG("Finding possible place positions");
    findPossiblePlacePoses(poses, height_above_table, delta_height, num_heights);
    visualize(poses);
    if(!setupTables())
      return false;
    group_.setSupportSurfaceName("table");
    return group_.place(object, poses);
  }

  std::vector<geometry_msgs::PoseStamped> generatePlacePoses(const object_recognition_msgs::TableArray &table_array,
							     double resolution, 
							     double height_above_table,
							     double delta_height = 0.01,
							     unsigned int num_heights = 2,
							     double min_distance_from_edge = 0.10) const
  {
    std::vector<geometry_msgs::PoseStamped> place_poses;
    // Assumption that the table's normal is along the Z axis
    for(std::size_t i=0; i < table_array.tables.size(); ++i)
    {
      if(table_array.tables[i].convex_hull.vertices.empty())
	continue;
      const int scale_factor = 100;
      std::vector<cv::Point2f> table_contour;
      for(std::size_t j=0; j < table_array.tables[i].convex_hull.vertices.size(); ++j)
	table_contour.push_back(cv::Point((table_array.tables[i].convex_hull.vertices[j].x-table_array.tables[i].x_min)*scale_factor, 
					  (table_array.tables[i].convex_hull.vertices[j].y-table_array.tables[i].y_min)*scale_factor));

      double x_range = fabs(table_array.tables[i].x_max-table_array.tables[i].x_min);
      double y_range = fabs(table_array.tables[i].y_max-table_array.tables[i].y_min);
      int max_range = (int) x_range + 1;
      if(max_range < (int) y_range + 1)
	max_range = (int) y_range + 1;

      int image_scale = std::max<int>(max_range, 4);
      cv::Mat src = cv::Mat::zeros(image_scale*scale_factor, image_scale*scale_factor, CV_8UC1);

      for(std::size_t j = 0; j < table_array.tables[i].convex_hull.vertices.size(); ++j )
      { 
	cv::line(src, table_contour[j],  table_contour[(j+1)%table_array.tables[i].convex_hull.vertices.size()], 
		  cv::Scalar( 255 ), 3, 8 ); 
      }

      unsigned int num_x = fabs(table_array.tables[i].x_max-table_array.tables[i].x_min)/resolution + 1;
      unsigned int num_y = fabs(table_array.tables[i].y_max-table_array.tables[i].y_min)/resolution + 1;

      ROS_DEBUG("Num points for possible place operations: %d %d", num_x, num_y);

      std::vector<std::vector<cv::Point> > contours;
      std::vector<cv::Vec4i> hierarchy;
      cv::findContours(src, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

      for(std::size_t j=0; j < num_x; ++j)
      {
	int point_x = j * resolution * scale_factor;
	for(std::size_t k=0; k < num_y; ++k)
	{
	  for(std::size_t mm=0; mm < num_heights; ++mm)
	  {
	    int point_y = k * resolution * scale_factor;
	    cv::Point2f point(point_x, point_y);
	    double result = cv::pointPolygonTest(contours[0], point, true);
	    if((int) result >= (int) (min_distance_from_edge*scale_factor))
	    {
	      tf::Point point((double) (point_x)/scale_factor + table_array.tables[i].x_min, 
			      (double) (point_y)/scale_factor + table_array.tables[i].y_min, 
			      0.0);
	      tf::Pose pose;
	      tf::poseMsgToTF(table_array.tables[i].pose.pose, pose);
	      point = pose * point;
	      if(place_left_ && point.y() <= 0.0)
		continue;
	      if(!place_left_ && point.y() > 0.0)
		continue;
	      geometry_msgs::PoseStamped place_pose;
	      place_pose.pose.orientation.w = 1.0;
	      place_pose.pose.position.x = point.x();
	      place_pose.pose.position.y = point.y();
	      place_pose.pose.position.z = point.z() + height_above_table + mm * delta_height;
	      place_pose.header = table_array.tables[i].pose.header;
	      place_poses.push_back(place_pose);
	    }
	  }
	}
      }
    }
    visualize(place_poses);
    return place_poses;
  }

  void findPossiblePlacePoses(std::vector<geometry_msgs::PoseStamped> &poses, 
			      double height_above_table, 
			      double delta_height=0.01, 
			      unsigned int num_heights = 2)
  {
    if(table_dirty_)
    {  
      {
	boost::mutex::scoped_lock tlock(table_lock_);
	ROS_DEBUG("Generating place poses");
	place_poses_ = generatePlacePoses(table_array_, place_resolution_, height_above_table, delta_height, num_heights);
      } 
      table_dirty_ = false;
    }
    poses = place_poses_;
  }

  void tableCallback(const object_recognition_msgs::TableArrayPtr &msg)
  {
    if(!listen_tables_)
      return;
    boost::mutex::scoped_lock tlock(table_lock_);
    ROS_DEBUG("Table callback");
    table_array_ = *msg;
    transformTableArray(table_array_);
    table_dirty_ = true;
  }

  void transformTableArray(object_recognition_msgs::TableArray &table_array)
  {
    for(std::size_t i=0; i < table_array.tables.size(); ++i)
    {
      std::string original_frame = table_array.tables[i].pose.header.frame_id;
      std::string root_frame = group_.getPoseReferenceFrame();
      if(table_array.tables[i].convex_hull.vertices.empty())
	continue;
      std::string error_text;
      if(!tf_.waitForTransform(root_frame, 
			       original_frame,
			       table_array.tables[i].pose.header.stamp,
			       ros::Duration(0.5),
			       ros::Duration(0.01),
			       &error_text))
      {
	ROS_ERROR_STREAM("TF error: " << error_text);
	continue;
      }

      tf_.transformPose(root_frame, table_array.tables[i].pose, table_array.tables[i].pose);
      table_array.tables[i].pose.pose.position.z += 0.025;
      ROS_DEBUG_STREAM("Successfully transformed table array from " << original_frame << " to " << root_frame);
    }
  }

  bool place_left_;
  bool listen_tables_;
  tf::TransformListener tf_;
  ros::NodeHandle node_handle_;
  bool table_dirty_;
  moveit::planning_interface::MoveGroup& group_;
  moveit::planning_interface::PlanningSceneInterface& planning_scene_interface_;
  ros::Subscriber table_subscriber_;
  double place_resolution_;
  object_recognition_msgs::TableArray table_array_;
  std::vector<geometry_msgs::PoseStamped> place_poses_;
  boost::mutex table_lock_;
  ros::Publisher visualization_publisher_, collision_object_publisher_, recognition_trigger_publisher_, octomap_trigger_publisher_;
};

int main(int argc, char **argv)
{
  ros::init (argc, argv, "right_arm_pick_place");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  ros::NodeHandle nh;

  ros::WallDuration(2.0).sleep();
  
  moveit::planning_interface::MoveGroup group("right_arm");
  group.setPlanningTime(45.0);
  ROS_INFO("Started move group interface");

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  ROS_INFO("Started planning scene interface");

  PickPlaceGroup pg(group, planning_scene_interface);
  ROS_INFO("Started pick place group");
  
  // wait a bit for ros things to initialize
  ros::WallDuration(2.0).sleep();

  std::vector<double> joint_values(7,0.0);
  joint_values[0] = -2.01;
  joint_values[1] = 0.737;
  joint_values[2] = -2.00;
  joint_values[3] = -2.00;
  joint_values[4] = -2.5774;
  joint_values[5] = -1.184;
  joint_values[6] = -1.16;

  group.setJointValueTarget(joint_values);
  ROS_INFO("Sending group joint target");
  group.move();
  ROS_INFO("Done group move");
  
  const static unsigned int START = 0;
  const static unsigned int PICK = 1;
  const static unsigned int PLACE = 2;
  const static unsigned int CLEAR = 3;
  unsigned int try_place = 0;
  unsigned int state = START;

  std::vector<std::string> objects, object_ids;
  std::string bowl_id;
  bool tried_both = false;

  pg.listen_tables_ = true;
  pg.triggerDetection();
  pg.triggerDetection();
  pg.triggerDetection();

  ROS_INFO("Starting pick and place");
  while(ros::ok())
  {    
    sleep(2.0);
    if(state == START)
    {  
      objects.clear();
      object_ids.clear();
      pg.listen_tables_ = true;
      pg.triggerDetection();
      pg.triggerDetection();
      pg.triggerDetection();
      sleep(3.0);
      pg.listen_tables_ = false;
      //      objects = planning_scene_interface.getKnownObjectNamesInROI(0.3,-0.5,0.6,0.7,0.5,0.9,true);
      object_ids = planning_scene_interface.getKnownObjectNamesInROI(-2.0,-0.75,0.0,2.0,0.75,1.2,true,objects);

      bool found_bowl = false;
      for(std::size_t i = 0; i < objects.size(); ++i)
      {
	ROS_INFO("Found object: %s", objects[i].c_str());
	if(objects[i] == "18691")
	{  
	  found_bowl = true;
	  bowl_id = object_ids[i];
	}
      } 
      if(objects.empty() || !found_bowl)
      {
	ROS_INFO("Could not find recognized object in workspace");
	group.setJointValueTarget(joint_values);
	group.move();
	continue;
      }
      else 
      {
	state = PICK;
	continue;
      }
    }
    else if (state == PICK)
    {    
      ROS_INFO("Trying to pickup object: 18691");
      if(pg.pick(bowl_id))
      {  
	ROS_INFO("Done pick");
	ros::WallDuration(1.0).sleep();  
	state = PLACE;
	continue;
      }
      else
      {
	state = START;
	continue;
      }
    }
    else if (state == PLACE)
    {      
      if(pg.place(bowl_id, 0.01) && !tried_both)
      {
	tried_both = false;
	ROS_INFO("Done place");
	state = CLEAR;
	pg.place_left_ = !pg.place_left_;
	sleep(3.0);
	continue;
      }
      if(!tried_both)
      {
	pg.place_left_ = !pg.place_left_;
	try_place = 0;
	tried_both = true;
	continue;
      }
      ROS_ERROR("Could not place object: HELP");
      break;
    }
    else if (state == CLEAR)
    {
      group.setJointValueTarget(joint_values);
      group.move();
      state = START;
      continue;
    }
  }
  ROS_INFO("Waiting for Ctrl-C");
  ros::waitForShutdown();
  return 0;
}
