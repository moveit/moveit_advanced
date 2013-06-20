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

#include <collision_distance_field_display/arrows_display.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreQuaternion.h>
#include <rviz/ogre_helpers/shape.h>
#include <boost/math/constants/constants.hpp>


moveit_rviz_plugin::ArrowsDisplay::ArrowsDisplay(
    Ogre::SceneNode* parent,
    const Eigen::Vector4f& color)
  : color_(color)
  , node_(parent->createChildSceneNode())
{ }

moveit_rviz_plugin::ArrowsDisplay::~ArrowsDisplay()
{
  clear();
  node_->getParentSceneNode()->removeAndDestroyChild(node_->getName());
}

void moveit_rviz_plugin::ArrowsDisplay::clear()
{
  for (std::vector<Arrow>::iterator it = arrows_.begin() ; it != arrows_.end() ; ++it)
  {
    delete it->cyl_;
    delete it->cone_;
  }
  arrows_.clear();
}

static inline Ogre::Vector3 OVec3(const Eigen::Vector3d& v)
{
  return Ogre::Vector3(v.x(), v.y(), v.z());
}

static inline Ogre::Quaternion OQuat(const Eigen::Quaterniond& q)
{
  return Ogre::Quaternion(q.w(), q.x(), q.y(), q.z());
}

// Assumes length extends from origin towards the z axis
void moveit_rviz_plugin::ArrowsDisplay::addArrow(
      const Eigen::Vector3d& base,
      const Eigen::Vector3d& tip,
      const Eigen::Vector4f& color,
      double max_cone_diameter,
      double min_cylinder_diameter)
{
  Eigen::Vector3d dir = tip - base;
  double len = dir.norm();
  if (len < std::numeric_limits<double>::epsilon())
    return;

  dir *= 1.0/len;
  Eigen::Vector3d tmp(dir.y(), dir.z(), dir.x());
  Eigen::Vector3d tan0 = dir.cross(tmp);
  tan0.normalize();
  Eigen::Vector3d tan1 = tan0.cross(dir);
  
  
  // determine the size
  double cyl_diameter = max_cone_diameter * 0.5;
  double cone_diameter = max_cone_diameter;
  double cone_len = max_cone_diameter * 2.0;
  double cyl_len = len - cone_len;

  if (len < 3 * cone_len)
  {
    cone_len = len / 3.0;
    cyl_len = len - cone_len;
    cone_diameter = cone_len * 0.5;
    cyl_diameter = cone_diameter * 0.5;
  }
  if (cyl_diameter < min_cylinder_diameter)
  {
    cyl_diameter = min_cylinder_diameter;
    if (cone_diameter < cyl_diameter * 1.1)
      cone_diameter = cyl_diameter * 1.1;
    if (cone_diameter > max_cone_diameter)
      cone_diameter = max_cone_diameter;
  }

  Arrow a;
  a.cyl_node_ = node_->createChildSceneNode();
  a.cone_node_ = node_->createChildSceneNode();
  a.cyl_ = new rviz::Shape(rviz::Shape::Cylinder, node_->getCreator(), a.cyl_node_);
  a.cone_ = new rviz::Shape(rviz::Shape::Cone, node_->getCreator(), a.cone_node_);


  // unscaled cylinder length is from y=-0.5 to y=+0.5
  // unscaled cylinder diameter is 1
  a.cyl_->setScale(Ogre::Vector3(cyl_diameter, cyl_len, cyl_diameter));

  // unscaled cone has base at y=-0.5 and tip at y=0.5
  // unscaled cone diameter is 1
  a.cone_->setScale(Ogre::Vector3(cone_diameter, cone_len, cone_diameter));

  Ogre::Quaternion cyl_quat(OVec3(tan0), OVec3(dir), OVec3(tan1));
  Ogre::Quaternion cone_quat(OVec3(tan0), OVec3(dir), OVec3(tan1));

  Eigen::Vector3d cyl_pos = base + 0.5 * cyl_len * dir;
  Eigen::Vector3d cone_pos = base + (cyl_len + 0.5 * cone_len) * dir;


  a.cyl_node_->setPosition(OVec3(cyl_pos));
  a.cyl_node_->setOrientation(cyl_quat);
  
  a.cone_node_->setPosition(OVec3(cone_pos));
  a.cone_node_->setOrientation(cone_quat);

  a.cyl_->setColor(color.x(), color.y(), color.z(), color.w());
  a.cone_->setColor(color.x(), color.y(), color.z(), color.w());

  arrows_.push_back(a);
}


