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

#ifndef COLLISION_DISTANCE_FIELD_DISPLAY_CYLINDERS_DISPLAY_H
#define COLLISION_DISTANCE_FIELD_DISPLAY_CYLINDERS_DISPLAY_H

#include <Eigen/Geometry>
#include <vector>

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class Shape;
}

namespace moveit_rviz_plugin
{

/// Represents some cylinders to draw in rviz.
class ArrowsDisplay
{
public:
  ArrowsDisplay(Ogre::SceneNode* parent,
             const Eigen::Vector4f& color = Eigen::Vector4f(1,1,1,1));
  ~ArrowsDisplay();

  void clear();

  // add arrow pointing from base to tip
  void addArrow(const Eigen::Vector3d& base,
                const Eigen::Vector3d& tip,
                double max_cone_diameter = 0.05,
                double min_cylinder_diameter = 0.001)
  {
    addArrow(base, tip, color_, max_cone_diameter, min_cylinder_diameter);
  }
  void addArrow(const Eigen::Vector3d& base,
                    const Eigen::Vector3d& tip,
                    const Eigen::Vector4f& color,
                    double max_cone_diameter = 0.05,
                    double min_cylinder_diameter = 0.001);

private:

  struct Arrow
  {
    rviz::Shape *cyl_;
    rviz::Shape *cone_;
    Ogre::SceneNode* cyl_node_;
    Ogre::SceneNode* cone_node_;
  };

  Ogre::SceneNode* node_;
  std::vector<Arrow> arrows_;
  Eigen::Vector4f color_;
};



}

#endif
