/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Acorn Pooley */

#include <collision_distance_field_display/collision_distance_field_display.h>
#include <collision_distance_field_display/shapes_display.h>
#include <collision_distance_field_display/color_cast.h>
#include "dfexamine.h"

#include <moveit/collision_detection_distance_field/static_distance_field.h>

#include <rviz/properties/color_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/float_property.h>

moveit_rviz_plugin::CollisionDistanceFieldDisplay::DFExamine::DFExamine(
      const char *descrip,
      const char *link_name,
      Ogre::SceneNode* parent,
      const Eigen::Affine3d& transform_to_node,
      const collision_detection::StaticDistanceField *df,
      int idx,
      const Eigen::Vector4f& color,
      const Eigen::Vector4f& near_color,
      double size)
  : spheres_(parent, color)
{
  if (!df->isCellIdValid(idx))
    return;

  int x,y,z;
  Eigen::Vector3d pt;
  double d;

  df->getCellPosition(idx, x, y, z);
  df->getCellPosition(idx, pt);
  const collision_detection::DistPosEntry& entry = df->getCell(idx);
  d = entry.distance_;

  int nidx = entry.cell_id_;
  int nx = -1;
  int ny = -1;
  int nz = -1;
  Eigen::Vector3d npt(0,0,0);
  double nd = 0;

  if (df->isCellIdValid(nidx))
  {
    df->getCellPosition(nidx, nx, ny, nz);
    df->getCellPosition(nidx, npt);
    const collision_detection::DistPosEntry& n_entry = df->getCell(nidx);
    nd = entry.distance_;
  }
  else
  {
    nidx = -1;
  }

  ROS_INFO("%s%s: idx=%5d (%3d, %3d, %3d) d=%7.3f   near=%5d (%3d, %3d, %3d) d=%7.3f  (%7.3f, %7.3f, %7.3f) (%7.3f, %7.3f, %7.3f)  max=(%3d,%3d,%3d)",
    descrip,
    link_name,
    idx,
    x,y,z,d,
    nidx,
    nx,ny,nz,nd,
    pt.x(), pt.y(), pt.z(),
    npt.x(), npt.y(), npt.z(),
    df->getNumCells(distance_field::DIM_X),
    df->getNumCells(distance_field::DIM_Y),
    df->getNumCells(distance_field::DIM_Z));

  spheres_.addSphere(
              transform_to_node * pt,
              size * 0.5,
              color);
  if (nidx != -1)
  {
    spheres_.addSphere(
                transform_to_node * npt,
                size * 0.5 + 0.001,
                near_color);
  }
}


moveit_rviz_plugin::CollisionDistanceFieldDisplay::DFExamine *moveit_rviz_plugin::CollisionDistanceFieldDisplay::examineDF(
      const char *descrip,
      const char *link_name,
      const collision_detection::StaticDistanceField *df,
      Ogre::SceneNode *node,
      const Eigen::Affine3d& transform_to_node) const
{
  if (!df_point_examine_enable_->getBool() || !df)
    return NULL;

  int x = df_point_examine_x_->getInt();
  int y = df_point_examine_y_->getInt();
  int z = df_point_examine_z_->getInt();
  if (!df->isCellValid(x,y,z))
    return NULL;

  int idx = df->getCellId(x,y,z);

  return new moveit_rviz_plugin::CollisionDistanceFieldDisplay::DFExamine(
                descrip,
                link_name,
                node ? node : planning_scene_node_,
                transform_to_node,
                df,
                idx,
                color_cast::getColorf(df_point_examine_color_),
                color_cast::getColorf(df_point_examine_near_color_),
                df_point_examine_size_->getFloat());
}
