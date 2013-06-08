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

#include <collision_distance_field_display/cylinders_display.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreQuaternion.h>
#include <rviz/ogre_helpers/shape.h>
#include <boost/math/constants/constants.hpp>


moveit_rviz_plugin::CylindersDisplay::CylindersDisplay(
    Ogre::SceneNode* parent,
    const Eigen::Vector4f& color)
  : color_(color)
  , node_(parent->createChildSceneNode())
{ }

moveit_rviz_plugin::CylindersDisplay::~CylindersDisplay()
{
  clear();
  node_->getParentSceneNode()->removeAndDestroyChild(node_->getName());
}

void moveit_rviz_plugin::CylindersDisplay::clear()
{
  for (std::vector<Cylinder>::iterator it = cyls_.begin() ; it != cyls_.end() ; ++it)
    delete it->shape_;
  cyls_.clear();
}

// Assumes length extends from origin towards the z axis
void moveit_rviz_plugin::CylindersDisplay::addZCylinder(
    const Eigen::Affine3d& pose, 
    double radius, 
    double length, 
    const Eigen::Vector4f& color)
{
  Cylinder c;
  c.node_ = node_->createChildSceneNode();
  c.shape_ = new rviz::Shape(rviz::Shape::Cylinder, node_->getCreator(), c.node_);

  // unscaled cylinder length is from origin to y=1
  // unscaled cylinder diameter is 1
  double diam = 2.0 * radius;
  c.shape_->setScale(Ogre::Vector3(diam, length, diam));

  Ogre::Vector3 position(pose.translation().x(), pose.translation().y(), pose.translation().z());
  Eigen::Quaterniond q(pose.rotation());
  Ogre::Quaternion orientation(q.w(), q.x(), q.y(), q.z());

  // rotate the cylinder from z-axis to y-axis
  Ogre::Quaternion fix(Ogre::Radian(boost::math::constants::pi<double>()/2.0), Ogre::Vector3(1.0, 0.0, 0.0));
  orientation = orientation * fix;

  c.node_->setPosition(position);
  c.node_->setOrientation(orientation);

  c.shape_->setColor(color.x(), color.y(), color.z(), color.w());

  cyls_.push_back(c);
}


