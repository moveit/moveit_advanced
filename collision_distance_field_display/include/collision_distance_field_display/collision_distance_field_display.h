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

#ifndef COLLISION_DISTANCE_FIELD_DISPLAY__COLLISION_DISTANCE_FIELD_DISPLAY
#define COLLISION_DISTANCE_FIELD_DISPLAY__COLLISION_DISTANCE_FIELD_DISPLAY

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
class StaticDistanceField;
}

namespace robot_sphere_representation
{
class RobotSphereRepresentation;
}

namespace mesh_ros
{
class RvizMeshShape;
}

namespace moveit_rviz_plugin
{
class PerLinkObjList;
class ShapesDisplay;

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

  const boost::shared_ptr<PerLinkObjList>& getLinkObjects() { return per_link_objects_; }
  const boost::shared_ptr<robot_sphere_representation::RobotSphereRepresentation>& getRobotSphereRepresentation() { return robot_sphere_rep_; }

  const collision_detection::CollisionRobotDistanceField *getCollisionRobotDistanceField() const;
  const collision_detection::CollisionWorldDistanceField *getCollisionWorldDistanceField() const;

  // Update global and per-link property values to match actual values in RobotSphereRepresentation.
  void updateLinkSphereGenPropertyValues();
  void updateAllSphereGenPropertyValues();

  enum {
    CD_UNKNOWN,
    CD_FCL,
    CD_DISTANCE_FIELD,
    CD_CNT
  };

  static std::string COLLISION_METHOD_STRING_FCL;
  static std::string COLLISION_METHOD_STRING_DISTANCE_FIELD;

  // if df_point_examine_ is enabled, show the point in this distance field
  // Returns NULL or a newly allocated DFExamine which should be placed in a shared pointer.
  class DFExamine;
  DFExamine *examineDF(const char *descrip,
                       const char *link_name,
                       const collision_detection::StaticDistanceField *df,
                       Ogre::SceneNode *node = NULL,
                       const Eigen::Affine3d& transform_to_node = Eigen::Affine3d::Identity()) const;

  // debug iteration property value
  int getDebugIteration() const;

  rviz::DisplayContext* getDisplayContext() { return context_; }

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
  void changedCollisionMethod();      // collision detection type (FCL, distance field, etc) changed
  void showCollidingSpheresChanged();
  void dfPointExamineChanged();

  void changedSphereGenMethod();
  void changedSphereQualMethod();
  void changedSphereGenResolution();
  void changedSphereGenTolerance();
  void changedRequestedNspheres();
  void changedSaveSpheresToSrdf();

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

  // get list of links that are in collision
  void getCollidingLinks(planning_scene_monitor::LockedPlanningSceneRO& ps,
                         std::vector<std::string> &links,
                         const robot_state::RobotState &kstate,
                         const collision_detection::AllowedCollisionMatrix& acm) const;

  // draw contact points and normals
  void showContactPoints(const robot_state::RobotState& state,
                         planning_scene_monitor::LockedPlanningSceneRO& ps);

  // draw contact points and normals (when using distance field collision
  void showContactPointsDF(const collision_detection::CollisionRobotDistanceField *crobot,
                           const robot_state::RobotState& state,
                           planning_scene_monitor::LockedPlanningSceneRO& ps);

  // draw collision distance arrow
  void showCollisionDistance(const robot_state::RobotState& state,
                             planning_scene_monitor::LockedPlanningSceneRO& ps);

  // save 
  void saveSpheresToSrdf();

  // Add per link data displays.
  void addPerLinkData();
  void addSphereGenProperties(rviz::Property* parent_property);

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
  rviz::BoolProperty* collision_df_use_spheres_;
  rviz::EditableEnumProperty* active_group_property_;
  rviz::BoolProperty* collision_aware_ik_property_;
  rviz::BoolProperty* publish_tf_property_;
  rviz::ColorProperty* colliding_link_color_property_;
  rviz::BoolProperty* contact_points_enable_property_;
  rviz::ColorProperty* contact_points_color_property_;
  rviz::FloatProperty* contact_points_size_property_;
  rviz::FloatProperty* contact_normal_length_property_;
  rviz::BoolProperty* colliding_spheres_enable_property_;
  rviz::ColorProperty* colliding_sphere_color_property_;
  rviz::FloatProperty* colliding_sphere_alpha_property_;
  rviz::BoolProperty* closest_distance_enable_property_;
  rviz::FloatProperty* closest_distance_value_property_;
  rviz::ColorProperty* closest_distance_color_nocollide_property_;
  rviz::ColorProperty* closest_distance_color_collide_property_;
  rviz::FloatProperty* closest_distance_alpha_property_;
  rviz::ColorProperty* joint_violation_link_color_property_;
  rviz::ColorProperty* attached_object_color_property_;
  rviz::FloatProperty* robot_alpha_property_;
  rviz::Property* collision_detection_category_;
  rviz::Property* sphere_gen_category_;
  rviz::Property* mesh_vis_category_;
  rviz::BoolProperty* save_to_srdf_property_;
  rviz::EnumProperty* sphere_gen_method_property_;
  rviz::EnumProperty* sphere_qual_method_property_;
  rviz::FloatProperty* sphere_gen_resolution_property_;
  rviz::FloatProperty* sphere_gen_tolerance_property_;
  rviz::IntProperty* requested_nspheres_property_;
  rviz::BoolProperty* df_point_examine_enable_;
  rviz::IntProperty* df_point_examine_x_;
  rviz::IntProperty* df_point_examine_y_;
  rviz::IntProperty* df_point_examine_z_;
  rviz::ColorProperty* df_point_examine_color_;
  rviz::ColorProperty* df_point_examine_near_color_;
  rviz::FloatProperty* df_point_examine_size_;
  rviz::IntProperty* debug_iteration_;

  // per link visible objects to display
  boost::shared_ptr<PerLinkObjList> per_link_objects_;

  // object for calculating spheres
  boost::shared_ptr<robot_sphere_representation::RobotSphereRepresentation> robot_sphere_rep_;

  bool unsetting_property_;  // true to skip callback when a property changes

  bool saving_spheres_to_srdf_; // true to trigger saving spheres to SRDF

  // for displaying contact points and colliding spheres 
  boost::shared_ptr<ShapesDisplay> contact_points_display_;
  boost::shared_ptr<ShapesDisplay> colliding_spheres_display_;
  boost::shared_ptr<ShapesDisplay> distance_display_;

  boost::shared_ptr<mesh_ros::RvizMeshShape> mesh_shape_;
};

}

#endif
