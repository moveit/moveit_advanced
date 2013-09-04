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

#include <moveit/collision_detection_distance_field/static_distance_field.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/body_operations.h>
#include <console_bridge/console.h>
#include "aabb.h"
#include <moveit/distance_field/propagation_distance_field.h>
#include <cmath>

void collision_detection::StaticDistanceField::determineCollisionPoints(
      const bodies::Body& body,
      double resolution,
      EigenSTL::vector_Vector3d& points)
{
  bodies::BoundingSphere sphere;
  body.computeBoundingSphere(sphere);
  double xval_s = std::floor((sphere.center.x() - sphere.radius - resolution) / resolution) * resolution;
  double yval_s = std::floor((sphere.center.y() - sphere.radius - resolution) / resolution) * resolution;
  double zval_s = std::floor((sphere.center.z() - sphere.radius - resolution) / resolution) * resolution;
  double xval_e = sphere.center.x() + sphere.radius + resolution;
  double yval_e = sphere.center.y() + sphere.radius + resolution;
  double zval_e = sphere.center.z() + sphere.radius + resolution;
  Eigen::Vector3d pt;
  for(pt.x() = xval_s; pt.x() <= xval_e; pt.x() += resolution) {
    for(pt.y() = yval_s; pt.y() <= yval_e; pt.y() += resolution) {
      for(pt.z() = zval_s; pt.z() <= zval_e; pt.z() += resolution) {
        if(body.containsPoint(pt)) {
          points.push_back(pt);
        }
      }
    }
  }
}


void collision_detection::StaticDistanceField::initialize(
      const bodies::Body& body,
      double resolution,
      double space_around_body,
      bool save_points)
{
  points_.clear();
  inv_twice_resolution_ = 1.0 / (2.0 * resolution);


  logInform("    create points at res=%f",resolution);
  EigenSTL::vector_Vector3d points;
  determineCollisionPoints(body, resolution, points);

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

  logInform("    space_around_body = %f",space_around_body);
  logInform("    DF: min=(%7.3f %7.3f %7.3f)  max=(%7.3f %7.3f %7.3f) (pre-space)",
                              aabb.min_.x(),
                              aabb.min_.y(),
                              aabb.min_.z(),
                              aabb.max_.x(),
                              aabb.max_.y(),
                              aabb.max_.z());

  aabb.min_ -= Eigen::Vector3d(space_around_body, space_around_body, space_around_body);
  aabb.max_ += Eigen::Vector3d(space_around_body, space_around_body, space_around_body);

  logInform("    DF: min=(%7.3f %7.3f %7.3f)  max=(%7.3f %7.3f %7.3f) (pre-adjust)",
                              aabb.min_.x(),
                              aabb.min_.y(),
                              aabb.min_.z(),
                              aabb.max_.x(),
                              aabb.max_.y(),
                              aabb.max_.z());

  aabb.min_.x() = std::floor(aabb.min_.x() / resolution) * resolution;
  aabb.min_.y() = std::floor(aabb.min_.y() / resolution) * resolution;
  aabb.min_.z() = std::floor(aabb.min_.z() / resolution) * resolution;

  logInform("    DF: min=(%7.3f %7.3f %7.3f)  max=(%7.3f %7.3f %7.3f) (post-adjust)",
                              aabb.min_.x(),
                              aabb.min_.y(),
                              aabb.min_.z(),
                              aabb.max_.x(),
                              aabb.max_.y(),
                              aabb.max_.z());

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

  logInform("    copy %d points.",
    getNumCells(distance_field::DIM_X) *
    getNumCells(distance_field::DIM_Y) *
    getNumCells(distance_field::DIM_Z));

  int pdf_x,pdf_y,pdf_z;
  int sdf_x,sdf_y,sdf_z;
  Eigen::Vector3d pdf_p, sdf_p;
  df.worldToGrid(aabb.min_.x(), aabb.min_.y(), aabb.min_.z(), pdf_x,pdf_y,pdf_z);
  worldToGrid(aabb.min_.x(), aabb.min_.y(), aabb.min_.z(), sdf_x,sdf_y,sdf_z);
  df.gridToWorld(pdf_x,pdf_y,pdf_z, pdf_p.x(), pdf_p.y(), pdf_p.z());
  gridToWorld(sdf_x,sdf_y,sdf_z, sdf_p.x(), sdf_p.y(), sdf_p.z());

  logInform("    DF: min=(%10.6f %10.6f %10.6f)  quant->%3d %3d %3d  (pdf)",
                              aabb.min_.x(),
                              aabb.min_.y(),
                              aabb.min_.z(),
                              pdf_x,
                              pdf_y,
                              pdf_z);
  logInform("    DF: min=(%10.6f %10.6f %10.6f)  quant<-%3d %3d %3d  (pdf)",
                              pdf_p.x(),
                              pdf_p.y(),
                              pdf_p.z(),
                              pdf_x,
                              pdf_y,
                              pdf_z);
  logInform("    DF: min=(%10.6f %10.6f %10.6f)  quant<-%3d %3d %3d  (sdf)",
                              sdf_p.x(),
                              sdf_p.y(),
                              sdf_p.z(),
                              sdf_x,
                              sdf_y,
                              sdf_z);


  df.worldToGrid(0,0,0, pdf_x,pdf_y,pdf_z);
  worldToGrid(0,0,0, sdf_x,sdf_y,sdf_z);
  df.gridToWorld(pdf_x,pdf_y,pdf_z, pdf_p.x(), pdf_p.y(), pdf_p.z());
  gridToWorld(sdf_x,sdf_y,sdf_z, sdf_p.x(), sdf_p.y(), sdf_p.z());

  logInform("    DF: org=(%10.6f %10.6f %10.6f)  quant->%3d %3d %3d  (pdf)",
                              0.0,
                              0.0,
                              0.0,
                              pdf_x,
                              pdf_y,
                              pdf_z);
  logInform("    DF: org=(%10.6f %10.6f %10.6f)  quant<-%3d %3d %3d  (pdf)",
                              pdf_p.x(),
                              pdf_p.y(),
                              pdf_p.z(),
                              pdf_x,
                              pdf_y,
                              pdf_z);
  logInform("    DF: org=(%10.6f %10.6f %10.6f)  quant<-%3d %3d %3d  (sdf)",
                              sdf_p.x(),
                              sdf_p.y(),
                              sdf_p.z(),
                              sdf_x,
                              sdf_y,
                              sdf_z);


  df.worldToGrid(points[0].x(), points[0].y(), points[0].z(), pdf_x,pdf_y,pdf_z);
  worldToGrid(points[0].x(), points[0].y(), points[0].z(), sdf_x,sdf_y,sdf_z);
  df.gridToWorld(pdf_x,pdf_y,pdf_z, pdf_p.x(), pdf_p.y(), pdf_p.z());
  gridToWorld(sdf_x,sdf_y,sdf_z, sdf_p.x(), sdf_p.y(), sdf_p.z());

  logInform("    DF: p0 =(%10.6f %10.6f %10.6f)  quant->%3d %3d %3d  (pdf)",
                              points[0].x(),
                              points[0].y(),
                              points[0].z(),
                              pdf_x,
                              pdf_y,
                              pdf_z);
  logInform("    DF: p0 =(%10.6f %10.6f %10.6f)  quant<-%3d %3d %3d  (pdf)",
                              pdf_p.x(),
                              pdf_p.y(),
                              pdf_p.z(),
                              pdf_x,
                              pdf_y,
                              pdf_z);
  logInform("    DF: p0 =(%10.6f %10.6f %10.6f)  quant<-%3d %3d %3d  (sdf)",
                              sdf_p.x(),
                              sdf_p.y(),
                              sdf_p.z(),
                              sdf_x,
                              sdf_y,
                              sdf_z);


  for (int z = 0 ; z < df.getZNumCells() ; ++z)
  {
    for (int y = 0 ; y < df.getYNumCells() ; ++y)
    {
      for (int x = 0 ; x < df.getXNumCells() ; ++x)
      {
        DistPosEntry entry;
        double dist = df.getDistance(x, y, z);
        const distance_field::PropDistanceFieldVoxel& voxel = df.getCell(x,y,z);

        if (dist < 0)
        {

          // propogation distance field has a bias of -1*resolution on points inside the object
          if (dist <= -resolution)
          {
            dist += resolution;
          }
          else
          {
            logError("PropagationDistanceField returned distance=%f between 0 and -resolution=%f."
                     "  Did someone fix it?"
                     "  Need to remove workaround from static_distance_field.cpp",
                     dist,-resolution);
            dist = 0.0;
          }
          entry.distance_ = dist;
          entry.cell_id_ = getCellId(
                              voxel.closest_negative_point_.x(),
                              voxel.closest_negative_point_.y(),
                              voxel.closest_negative_point_.z());
        }
        else
        {
          entry.distance_ = dist;
          entry.cell_id_ = getCellId(
                              voxel.closest_point_.x(),
                              voxel.closest_point_.y(),
                              voxel.closest_point_.z());
        }
        setCell(x, y, z, entry);
      }
    }
  }

  if (save_points)
    std::swap(points, points_);
}

void collision_detection::StaticDistanceField::getCellGradient(
      int cell_id,
      Eigen::Vector3d& gradient) const
{
    if (cell_id + this->stride2_ + this->stride1_ + 1 >= num_cells_total_ ||
      cell_id - this->stride2_ - this->stride1_ - 1 < 0)
  {
    gradient = Eigen::Vector3d(1,0,0);
    return;
  }

  gradient.x() = (getCell(cell_id + stride1_).distance_ - getCell(cell_id - stride1_).distance_) * inv_twice_resolution_;
  gradient.y() = (getCell(cell_id + stride2_).distance_ - getCell(cell_id - stride2_).distance_) * inv_twice_resolution_;
  gradient.z() = (getCell(cell_id +        1).distance_ - getCell(cell_id -        1).distance_) * inv_twice_resolution_;
}
