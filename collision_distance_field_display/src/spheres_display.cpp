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

#include <collision_distance_field_display/spheres_display.h>
#include <OGRE/OgreSceneNode.h>
#include <rviz/ogre_helpers/shape.h>


moveit_rviz_plugin::SpheresDisplay::SpheresDisplay(
    Ogre::SceneNode* parent,
    const Eigen::Vector4f& color)
  : color_(color)
  , node_(parent->createChildSceneNode())
{
}

moveit_rviz_plugin::SpheresDisplay::~SpheresDisplay()
{
  clear();
  node_->getParentSceneNode()->removeAndDestroyChild(node_->getName());
}

void moveit_rviz_plugin::SpheresDisplay::clear()
{
  for (std::vector<Sphere>::iterator it = spheres_.begin() ; it != spheres_.end() ; ++it)
    delete it->shape_;
  spheres_.clear();
}

void moveit_rviz_plugin::SpheresDisplay::addSphere(
    const Eigen::Vector3d& center, 
    double radius, 
    const Eigen::Vector4f& color)
{
  Sphere s;
  s.node_ = node_->createChildSceneNode();
  s.shape_ = new rviz::Shape(rviz::Shape::Sphere, node_->getCreator(), s.node_);

  Ogre::Vector3 pos(center.x(), center.y(), center.z());
  Ogre::Vector3 scale(radius*2, radius*2, radius*2);
  s.node_->setPosition(pos);
  s.shape_->setScale(scale);
  s.shape_->setColor(color.x(), color.y(), color.z(), color.w());

  spheres_.push_back(s);
}

