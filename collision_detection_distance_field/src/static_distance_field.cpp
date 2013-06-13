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
 *   * Neither the name of the Willow Garage nor the names of its
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

#include <moveit/collision_detection_distance_field/static_distance_field.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/body_operations.h>
#include <console_bridge/console.h>
#include "aabb.h"
#include <moveit/distance_field/distance_field_common.h>
#include <moveit/distance_field/propagation_distance_field.h>

void collision_detection::StaticDistanceField::initialize(
      const bodies::Body& body,
      double resolution,
      double space_around_body)
{
  logInform("    create points at res=%f",resolution);
  EigenSTL::vector_Vector3d points = distance_field::determineCollisionPoints(&body, resolution);

  if (points.empty())
  {
    logWarn("    StaticDistanceField::initialize: No points in body. Using origin.");
    points.push_back(body.getPose().translation());

    if (body.getType() == shapes::MESH)
    {
      const bodies::ConvexMesh& mesh = dynamic_cast<const bodies::ConvexMesh&>(body);
      const EigenSTL::vector_Vector3d& verts = mesh.getVertices();
      logWarn("    StaticDistanceField::initialize: also using %d vertices.", int(verts.size()));

      EigenSTL::vector_Vector3d::const_iterator it = verts.begin();
      EigenSTL::vector_Vector3d::const_iterator it_end = verts.end();
      for ( ; it != it_end ; ++it)
      {
        points.push_back(*it);
      }
    }
  }
  logInform("    StaticDistanceField::initialize: Using %d points.", points.size());

  AABB aabb;
  aabb.add(points);

  aabb.min_ -= Eigen::Vector3d(space_around_body, space_around_body, space_around_body);
  aabb.max_ += Eigen::Vector3d(space_around_body, space_around_body, space_around_body);
  Eigen::Vector3d size = aabb.max_ - aabb.min_;

  double diagonal = size.norm();

  logInform("    DF: sz=(%7.3f %7.3f %7.3f) cnt=(%d %d %d) diag=%f",
                              size.x(),
                              size.y(),
                              size.z(),
                              int(size.x()/resolution),
                              int(size.y()/resolution),
                              int(size.z()/resolution),
                              diagonal);

    
  distance_field::PropagationDistanceField df(
                              size.x(),
                              size.y(),
                              size.z(),
                              resolution,
                              aabb.min_.x(),
                              aabb.min_.y(),
                              aabb.min_.z(),
                              diagonal * 2.0,
                              true);
  df.addPointsToField(points);

  DistPosEntry default_entry;
  default_entry.distance_ = diagonal * 2.0;
  default_entry.cell_id_ = -1;

  resize(size.x(),
         size.y(),
         size.z(),
         resolution,
         aabb.min_.x(),
         aabb.min_.y(),
         aabb.min_.z(),
         default_entry);

  for (int z = 0 ; z < df.getZNumCells() ; ++z)
  {
    for (int y = 0 ; y < df.getYNumCells() ; ++y)
    {
      for (int x = 0 ; x < df.getXNumCells() ; ++x)
      {
        DistPosEntry entry;
        double dist = df.getDistance(x, y, z);

        // propogation distance field has a bias of -1 on internal points
        dist = (dist <= -1.0) ? (dist+1.0) : dist;

        entry.distance_ = d;
        entry.cell_id_ = getCellId(x,y,z);
        setCell(x, y, z, entry);
      }
    }
  }
}

