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

#ifndef COLLISION_DISTANCE_FIELD_DISPLAY_POINTS_DISPLAY_H
#define COLLISION_DISTANCE_FIELD_DISPLAY_POINTS_DISPLAY_H

#include <Eigen/Core>
#include <rviz/ogre_helpers/point_cloud.h>

namespace Ogre
{
class SceneNode;
}

namespace moveit_rviz_plugin
{

/// Represents some points to draw in rviz.
class PointsDisplay
{
public:
  PointsDisplay(Ogre::SceneNode* parent,
                const Eigen::Vector4f& color = Eigen::Vector4f(1,1,1,1),
                double point_size = 0.005);
  ~PointsDisplay();

  // remove all points from the display
  void clear();

  // add a point to the display
  void addPoint(const Eigen::Vector3d& point)
  {
    addPoint(point, color_);
  }
  void addPoint(const Eigen::Vector3d& point, const Eigen::Vector4f& color);

  // add several points to the display
  void addPoints(rviz::PointCloud::Point *p, std::size_t cnt);

private:
  rviz::PointCloud* points_;
  Ogre::SceneNode* node_;
  Eigen::Vector4f color_;
};


}

#endif
