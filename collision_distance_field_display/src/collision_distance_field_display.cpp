/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Acorn Pooley */

#include <collision_distance_field_display/collision_distance_field_display.h>
#include <collision_distance_field_display/color_cast.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <eigen_conversions/eigen_msg.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/editable_enum_property.h>
#include <rviz/frame_manager.h>
#include <rviz/display_factory.h>

#include <moveit/rviz_plugin_render_tools/robot_state_visualization.h>

moveit_rviz_plugin::CollisionDistanceFieldDisplay::CollisionDistanceFieldDisplay()
  : PlanningSceneDisplay()
  , robot_model_loaded_(false)
  , robot_visual_dirty_(true)
  , robot_visual_position_dirty_(true)
  , robot_markers_dirty_(true)
  , robot_markers_position_dirty_(true)
  , int_marker_display_(NULL)
{
  show_robot_visual_property_ = new rviz::BoolProperty(
                                      "Show Robot Visual",
                                      false,
                                      "Show the robot visual appearance (what it looks like).",
                                      this,
                                      SLOT( robotVisualChanged() ));
  show_robot_collision_property_ = new rviz::BoolProperty(
                                      "Show Robot Collision",
                                      true,
                                      "Show the robot collision geometry.",
                                      this,
                                      SLOT( robotVisualChanged() ));
  active_group_property_ = new rviz::EditableEnumProperty(
                                      "Active Group",
                                      "",
                                      "The name of the group of links to interact with (from the ones defined in the SRDF)",
                                      this,
                                      SLOT( changedActiveGroup() ),
                                      this );
  collision_aware_ik_property_ = new rviz::BoolProperty(
                                      "Collision aware IK",
                                      true,
                                      "Check collisions when doing IK.",
                                      this);
  publish_tf_property_ = new rviz::BoolProperty(
                                      "Publish Robot State on TF",
                                      true,
                                      "Enable publishing of robot link frames on TF (useful with TF display).",
                                      this);
  attached_object_color_property_ = new rviz::ColorProperty(
                                      "Attached Object Color",
                                      QColor( 204, 51, 204 ),
                                      "Color to draw objects attached to the robot.",
                                      this,
                                      SLOT( robotVisualChanged() ));
  robot_alpha_property_ = new rviz::FloatProperty(
                                      "Robot Alpha",
                                      1.0,
                                      "0 is fully transparent, 1.0 is fully opaque.",
                                      this,
                                      SLOT( robotVisualChanged() ));

  // turn Scene Robot visual off by default
  if (scene_robot_enabled_property_)
    scene_robot_enabled_property_->setValue(false);
}

moveit_rviz_plugin::CollisionDistanceFieldDisplay::~CollisionDistanceFieldDisplay()
{
  background_process_.setJobUpdateEvent(BackgroundProcessing::JobUpdateCallback());
  clearJobs();

  robot_visual_.reset();
  delete int_marker_display_;
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::onInitialize()
{
  PlanningSceneDisplay::onInitialize();

  robot_visual_.reset(new RobotStateVisualization(planning_scene_node_, context_, "Robot", NULL));
  robotVisualChanged();

  delete int_marker_display_;
  int_marker_display_ = context_->getDisplayFactory()->make("rviz/InteractiveMarkers");
  int_marker_display_->initialize(context_);
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::onDisable()
{
  if (robot_interaction_)
    robot_interaction_->clear();
  int_marker_display_->setEnabled(false);
  robot_visual_->setVisible(false);
  PlanningSceneDisplay::onDisable();
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::onEnable()
{
  PlanningSceneDisplay::onEnable();
  robotVisualChanged();
  robotMarkersChanged();
  int_marker_display_->setEnabled(true);
  int_marker_display_->setFixedFrame(fixed_frame_);
  changedActiveGroup();
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::reset()
{
  robot_model_loaded_ = false;
  robot_visual_->clear();
  robot_visual_->setVisible(false);

  PlanningSceneDisplay::reset();
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::fixedFrameChanged()
{
  PlanningSceneDisplay::fixedFrameChanged();
  if (int_marker_display_)
    int_marker_display_->setFixedFrame(fixed_frame_);
  changedActiveGroup();
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::onRobotModelLoaded()
{
  PlanningSceneDisplay::onRobotModelLoaded();

  robot_interaction_.reset(new robot_interaction::RobotInteraction(getRobotModel(), "distance_field_display"));
  int_marker_display_->subProp("Update Topic")->setValue(QString::fromStdString(robot_interaction_->getServerTopic() + "/update"));
  robot_visual_->load(*getRobotModel()->getURDF());

  robot_state::RobotStatePtr state(new robot_state::RobotState(getRobotModel()));
  robot_state_handler_.reset(new robot_interaction::RobotInteraction::InteractionHandler("current", *state, planning_scene_monitor_->getTFClient()));
  robot_state_handler_->setUpdateCallback(boost::bind(&CollisionDistanceFieldDisplay::markersMoved, this, _1, _2));
  robot_state_handler_->setStateValidityCallback(boost::bind(&CollisionDistanceFieldDisplay::isIKSolutionCollisionFree, this, _1, _2));

  if (!active_group_property_->getStdString().empty() &&
      !getRobotModel()->hasJointModelGroup(active_group_property_->getStdString()))
  {
    active_group_property_->setStdString("");
  }

  const std::vector<std::string> &groups = getRobotModel()->getJointModelGroupNames();
  active_group_property_->clearOptions();
  for (std::size_t i = 0 ; i < groups.size() ; ++i)
    active_group_property_->addOptionStd(groups[i]);
  active_group_property_->sortOptions();
  if (!groups.empty() && active_group_property_->getStdString().empty())
    active_group_property_->setStdString(groups[0]);


  robot_model_loaded_ = true;
  robotVisualChanged();
  robotMarkersChanged();
  changedActiveGroup();
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::markersMoved(robot_interaction::RobotInteraction::InteractionHandler *, bool error_state_changed)
{
  robotMarkerPositionsChanged();
  robotVisualPositionChanged();
}

bool moveit_rviz_plugin::CollisionDistanceFieldDisplay::isIKSolutionCollisionFree(robot_state::JointStateGroup *group, const std::vector<double> &ik_solution) const
{
  if (collision_aware_ik_property_->getBool() && planning_scene_monitor_)
  {
    group->setVariableValues(ik_solution);
    bool res = !getPlanningSceneRO()->isStateColliding(*group->getRobotState(), group->getName());
    return res;
  }
  else
    return true;
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::changedActiveGroup()
{
  if (!robot_model_loaded_ || !robot_interaction_)
    return;

  if (active_group_property_->getStdString().empty())
  {
    robot_interaction_->clear();
  }
  else if (!getRobotModel()->hasJointModelGroup(active_group_property_->getStdString()))
  {
    robot_interaction_->clear();
  }
  else
  {
    robot_interaction_->decideActiveComponents(active_group_property_->getStdString(), robot_interaction::RobotInteraction::EEF_POSITION_AND_VIEWPLANE);
  }

  robotVisualChanged();
  robotMarkersChanged();
}

// call when the robot visual state changes to update the robot visual
void moveit_rviz_plugin::CollisionDistanceFieldDisplay::robotVisualChanged()
{
  robot_visual_dirty_ = true;
  robot_visual_position_dirty_ = true;
}

// call when the robot state changes to update the robot visual
void moveit_rviz_plugin::CollisionDistanceFieldDisplay::robotVisualPositionChanged()
{
  robot_visual_position_dirty_ = true;
}

// call when the appearance and position of the markers needs to change
void moveit_rviz_plugin::CollisionDistanceFieldDisplay::robotMarkersChanged()
{
  robot_markers_dirty_ = true;
  robot_markers_position_dirty_ = true;
  addBackgroundJob(boost::bind(&CollisionDistanceFieldDisplay::updateRobotMarkers, this), "updateRobotMarkers:all");
}

// call when only the position of the markers needs to change
void moveit_rviz_plugin::CollisionDistanceFieldDisplay::robotMarkerPositionsChanged()
{
  robot_markers_position_dirty_ = true;
  addBackgroundJob(boost::bind(&CollisionDistanceFieldDisplay::updateRobotMarkers, this), "updateRobotMarkers:position");
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::setRobotState(const robot_state::RobotState &state)
{
  robot_state_handler_->setState(state);
  robotVisualPositionChanged();
  robotMarkerPositionsChanged();
}

robot_state::RobotStateConstPtr moveit_rviz_plugin::CollisionDistanceFieldDisplay::getRobotState() const
{
  return robot_state_handler_->getState();
}



//===========================================================================
// update() processing
//===========================================================================

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::update(float wall_dt, float ros_dt)
{
  if (int_marker_display_)
    int_marker_display_->update(wall_dt, ros_dt);

  updateRobotVisual();

  PlanningSceneDisplay::update(wall_dt, ros_dt);

  if (publish_tf_property_->getBool())
    publishTF();
}

// Update the robot visual appearance based on attributes.
// Should only be called from update().  To trigger this call robotVisualChanged() or robotVisualPositionChanged()
void moveit_rviz_plugin::CollisionDistanceFieldDisplay::updateRobotVisual()
{
  if (!robot_model_loaded_)
    return;
  
  if (robot_visual_dirty_)
  {
    robot_visual_dirty_ = false;
    robot_visual_position_dirty_ = false;

    bool vis = show_robot_visual_property_->getBool();
    bool col = show_robot_collision_property_->getBool();

    robot_visual_->setAlpha(robot_alpha_property_->getFloat());
    
    robot_visual_->setCollisionVisible(col);
    robot_visual_->setVisualVisible(vis);
    robot_visual_->setVisible(isEnabled() && (vis || col));

    robot_visual_->update(getRobotState(), color_cast::getColorRGBA(attached_object_color_property_, robot_alpha_property_));

    context_->queueRender();
  }
  else if (robot_visual_position_dirty_)
  {
    robot_visual_position_dirty_ = false;
    robot_visual_->update(getRobotState());
    context_->queueRender();
  }
}

void moveit_rviz_plugin::CollisionDistanceFieldDisplay::publishTF()
{
  const bool PUBLISH_TF_AS_JOINT_TREE = true;

  if (!robot_model_loaded_)
    return;

  robot_state::RobotStateConstPtr state = getRobotState();
  if (!state)
    return;

  const std::vector<robot_state::LinkState*> &ls = state->getLinkStateVector();
  std::vector<geometry_msgs::TransformStamped> transforms(ls.size());
  const std::string &planning_frame = planning_scene_monitor_->getPlanningScene()->getPlanningFrame();
  std::size_t j = 0;
  ros::Time now = ros::Time::now();
  for (std::size_t i = 0 ; i < ls.size() ; ++i)
  {
    if (ls[i]->getName() == planning_frame)
      continue;
    const Eigen::Affine3d &t = ls[i]->getGlobalLinkTransform();
    const robot_state::LinkState* pls = ls[i]->getParentLinkState();
    if (PUBLISH_TF_AS_JOINT_TREE && pls)
    {
      const Eigen::Affine3d &pt = pls->getGlobalLinkTransform();
      Eigen::Affine3d rel = pt.inverse() * t;

      tf::transformEigenToMsg(rel, transforms[j].transform);
      transforms[j].header.frame_id = pls->getName();
    }
    else
    {
      tf::transformEigenToMsg(t, transforms[j].transform);
      transforms[j].header.frame_id = planning_frame;
    }
    transforms[j].header.stamp = now;
    transforms[j].child_frame_id = ls[i]->getName();
    ++j;
  }




  transforms.resize(j);
  tf_broadcaster_.sendTransform(transforms);
}

//===========================================================================
// background tasks -- these run on background thread
//===========================================================================

// Update the marker visual appearance based on the robot state.
// Do not call directly.  To trigger this call robotMarkersChanged() or robotMarkerPositionsChanged().
void moveit_rviz_plugin::CollisionDistanceFieldDisplay::updateRobotMarkers()
{
  if (!robot_interaction_)
    return;

  if (robot_markers_dirty_)
  {
    const static double DEFAULT_MARKER_SCALE = 0.0;
    robot_markers_dirty_ = false;
    robot_markers_position_dirty_ = false;
    robot_interaction_->clearInteractiveMarkers();
    robot_interaction_->addInteractiveMarkers(robot_state_handler_, DEFAULT_MARKER_SCALE);
    robot_interaction_->publishInteractiveMarkers();
  }
  else if (robot_markers_position_dirty_)
  {
    robot_markers_position_dirty_ = false;
    robot_interaction_->updateInteractiveMarkers(robot_state_handler_);
  }
}

