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

#include <collision_distance_field_display/points_display.h>
#include <rviz/ogre_helpers/point_cloud.h>
#include <OGRE/OgreSceneNode.h>



moveit_rviz_plugin::PointsDisplay::PointsDisplay(
    Ogre::SceneNode* parent,
    const Eigen::Vector4f& color,
    double point_size)
  : color_(color)
  , points_(new rviz::PointCloud())
  , node_(parent->createChildSceneNode())
{
  node_->attachObject(points_);
  points_->setRenderMode(rviz::PointCloud::RM_SQUARES);
  points_->setDimensions(point_size, point_size, 0.0f);
  points_->setAlpha(color.w(), false);
}

moveit_rviz_plugin::PointsDisplay::~PointsDisplay()
{
  node_->getParentSceneNode()->removeAndDestroyChild(node_->getName());
  delete points_;
}

void moveit_rviz_plugin::PointsDisplay::clear()
{
  points_->clear();
}

void moveit_rviz_plugin::PointsDisplay::addPoint(
    const Eigen::Vector3d& point, 
    const Eigen::Vector4f& color)
{
  rviz::PointCloud::Point p;
  p.position.x = point.x();
  p.position.y = point.y();
  p.position.z = point.z();
  p.color.r = color.x();
  p.color.g = color.y();
  p.color.b = color.z();
  p.color.a = color.w();
  points_->addPoints(&p, 1);
  if (color.w() != color_.w())
    points_->setAlpha(1.0, true);
}

void moveit_rviz_plugin::PointsDisplay::addPoints(
    rviz::PointCloud::Point *p,
    std::size_t cnt)
{
  points_->addPoints(p, cnt);
}
