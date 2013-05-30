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

#ifndef COLLISION_DISTANCE_FIELD_DISPLAY_COLLISION_DISTANCE_FIELD_DISPLAY_H
#define COLLISION_DISTANCE_FIELD_DISPLAY_COLLISION_DISTANCE_FIELD_DISPLAY_H

#include <moveit/planning_scene_rviz_plugin/planning_scene_display.h>

#ifndef Q_MOC_RUN
#include <tf/transform_broadcaster.h>
#include <moveit/robot_interaction/robot_interaction.h>
#endif

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class ColorProperty;
class FloatProperty;
class IntProperty;
class EnumProperty;
class EditableEnumProperty;
}

namespace collision_detection
{
class CollisionRobotDistanceField;
class CollisionWorldDistanceField;
}

namespace moveit_rviz_plugin
{

class PerPartObjList;

// Visualise collision distance field info.
class CollisionDistanceFieldDisplay: public PlanningSceneDisplay
{
Q_OBJECT
public:
  CollisionDistanceFieldDisplay();
  virtual ~CollisionDistanceFieldDisplay();

  // access to the robot state
  void setRobotState(const robot_state::RobotState &state);
  robot_state::RobotStateConstPtr getRobotState() const;

  const boost::shared_ptr<PerPartObjList>& getLinkObjects() { return per_link_objects_; }

  const collision_detection::CollisionRobotDistanceField *getCollisionRobotDistanceField() const;
  const collision_detection::CollisionWorldDistanceField *getCollisionWorldDistanceField() const;

protected:
  virtual void onInitialize();
  virtual void onEnable();
  virtual void onDisable();
  virtual void onRobotModelLoaded();
  virtual void reset();
  virtual void fixedFrameChanged();
  virtual void update(float wall_dt, float ros_dt);

private Q_SLOTS:
  void robotVisualChanged();          // call when the robot visual state changes to update the robot visual
  void robotVisualPositionChanged();  // call when the robot state changes to update the robot visual
  void robotMarkersChanged();         // call when the appearance and position of the markers needs to change
  void robotMarkerPositionsChanged(); // call when only the position of the markers needs to change
  void changedActiveGroup();
  void changedCollisionMethod();

private:

  // callback called when interactive markers have moved
  void markersMoved(robot_interaction::RobotInteraction::InteractionHandler *, bool error_state_changed);

  // true if solution is not in collision.
  // NOTE: this modifies the state that group is a child of.
  bool isIKSolutionCollisionFree(robot_state::JointStateGroup *group, const std::vector<double> &ik_solution) const;

  // Do not call this directly.  Instead call robotVisualChanged() which causes update() to call this.
  // Updates the state of the robot visual and queues a render.
  void updateRobotVisual();

  // Do not call this directly.  Instead call robotVisualChanged() which causes update() to call this.
  // Check collisions and joint limits and update colors.
  void updateLinkColors(const robot_state::RobotState& state);

  // Do not call this directly.  This is called regularly by update().
  // Publish the current robot state on TF.
  void publishTF();

  // Do not call this directly.  Instead call robotMarkersChanged() which causes this to be called in background.
  // Creates and/or updates the robot_interaction markers.
  void updateRobotMarkers();

  // Add per link data displays.
  void add_per_link_data(rviz::Property* parent_property);


  // for drawing the robot
  RobotStateVisualizationPtr robot_visual_;
  bool robot_visual_dirty_;
  bool robot_visual_position_dirty_;
  bool robot_model_loaded_;

  // for managing interactive markers to manipulate the robot state.
  rviz::Display *int_marker_display_;
  robot_interaction::RobotInteractionPtr robot_interaction_;
  robot_interaction::RobotInteraction::InteractionHandlerPtr robot_state_handler_;
  bool robot_markers_dirty_;
  bool robot_markers_position_dirty_;

  // for publishing robot state in standalone mode
  tf::TransformBroadcaster tf_broadcaster_;

  
  // User-editable property variables.
  rviz::Property* robot_state_category_;
  rviz::BoolProperty* show_robot_visual_property_;
  rviz::BoolProperty* show_robot_collision_property_;
  rviz::EnumProperty* collision_method_property_;
  rviz::EditableEnumProperty* active_group_property_;
  rviz::BoolProperty* collision_aware_ik_property_;
  rviz::BoolProperty* publish_tf_property_;
  rviz::ColorProperty* colliding_link_color_property_;
  rviz::ColorProperty* joint_violation_link_color_property_;
  rviz::ColorProperty* attached_object_color_property_;
  rviz::FloatProperty* robot_alpha_property_;

  // per link visible objects to display
  boost::shared_ptr<PerPartObjList> per_link_objects_;
};

}

#endif
