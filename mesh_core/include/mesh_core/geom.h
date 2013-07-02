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

#ifndef MESH_CORE__GEOM
#define MESH_CORE__GEOM

#include <eigen_stl_containers/eigen_stl_containers.h>

namespace mesh_core
{

/// append points to a vector.
// data is 3 doubles per point: x,y,z
// npoints are taken from data and pushed on the back of vector.
void appendPoints(
    EigenSTL::vector_Vector3d& vector,
    int npoints,
    const double *data);

/// append transformed points to a vector.
// data is 3 doubles per point: x,y,z
// npoints are taken from data, transformed by xform, and pushed on the back of
// vector.
void appendPointsTransformed(
    EigenSTL::vector_Vector3d& vector,
    const Eigen::Affine3d& xform,
    int npoints,
    const double *data);

class Plane
{
public:
  // construct from normal abc and d
  Plane(const Eigen::Vector3d& normal = Eigen::Vector3d(0,0,1),
        double d = 0.0);

  // construct from normal and point on plane
  Plane(const Eigen::Vector3d& normal, const Eigen::Vector3d& point_on_plane);

  // construct by least squares approximation over a set of points.
  Plane(const EigenSTL::vector_Vector3d& points);

  const Eigen::Vector3d& getNormal() const { return normal_; }
  double getA() const { return normal_.x(); }
  double getB() const { return normal_.y(); }
  double getC() const { return normal_.z(); }
  double getD() const { return d_; }

  // return signed distance of point from plane.
  // dist > 0 if normal points from plane to point
  // dist < 0 if normal points from plane away from point
  double dist(const Eigen::Vector3d& point) const;

protected:
  // set plane from least squares fit of points.
  // Optionally return average of points.
  void leastSquaresGeneral(const EigenSTL::vector_Vector3d& points,
                           Eigen::Vector3d* average = NULL);
  void leastSquaresFast(const EigenSTL::vector_Vector3d& points,
                        Eigen::Vector3d* average = NULL);

  // plane equation is normal_.dot(pt) + d_ = 0;
  Eigen::Vector3d normal_;
  double d_;
};

class PlaneProjection : Plane
{
public:
  // construct from normal, point on plane, and optional x axis
  PlaneProjection(const Eigen::Vector3d& normal,
                  const Eigen::Vector3d& origin,
                  const Eigen::Vector3d& x_axis = Eigen::Vector3d(1,0,0));

  // construct from plane, origin, and optional x axis
  // Default origin is arbitrary.
  PlaneProjection(const Plane& plane,
                  const Eigen::Vector3d& near_origin = Eigen::Vector3d(0,0,0),
                  const Eigen::Vector3d& x_axis = Eigen::Vector3d(1,0,0));

  // construct by least squares approximation over a set of points.
  // Default origin is average of points.
  PlaneProjection(const EigenSTL::vector_Vector3d& points,
                  const Eigen::Vector3d* near_origin = NULL,
                  const Eigen::Vector3d& x_axis = Eigen::Vector3d(1,0,0));

  // project to and from plane
  Eigen::Vector2d project(const Eigen::Vector3d& p) const;
  Eigen::Vector3d extract(const Eigen::Vector2d& p) const;

  void getFrame(Eigen::Affine3d& frame) const;
  Eigen::Quaterniond getOrientation() const;
  const Eigen::Vector3d& getOrigin() const { return origin_; }

private:
  void initMatrix(const Eigen::Vector3d& x_axis);

  Eigen::Vector3d x_axis_;
  Eigen::Vector3d y_axis_;
  Eigen::Vector3d origin_;
};

class LineSegment2D
{
public:
  // construct from 2 endpoints
  LineSegment2D(const Eigen::Vector2d& a = Eigen::Vector2d(0,0),
         const Eigen::Vector2d& b = Eigen::Vector2d(0,0));

  // change the endpoints
  void initialize(const Eigen::Vector2d& a = Eigen::Vector2d(0,0),
         const Eigen::Vector2d& b = Eigen::Vector2d(0,0));

  // calculate intersection of 2 line segments.
  // True if line segments intersect.
  // intersection is filled in whether segments intersect or
  // not.  If lines are parallel intersection is set to 0,0.
  // If non-NULL parallel returns true if lines are parallel.
  bool intersect(const LineSegment2D& other,
                 Eigen::Vector2d& intersection,
                 bool& parallel) const;

private:
  Eigen::Vector2d pt_[2];

  // true if x0 == x1 +/-epsilon
  bool vertical_;

  // if line is not vertical:  1/(x1 - x0)
  // if line is vertical:      1/(y1 - y0)
  // if line is a point:       0
  double inv_dx_;

  // valid only if vertical_ is false
  double slope_;
  double y_intercept_;

  //Eigen::Vector2d ab_normalized_;
  //double len_;
};

}

inline double mesh_core::Plane::dist(const Eigen::Vector3d& point) const
{
  return point.dot(normal_) + d_;
}


#endif

