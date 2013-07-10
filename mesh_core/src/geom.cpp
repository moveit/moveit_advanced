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

#include <mesh_core/geom.h>
#include <console_bridge/console.h>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

//###########################################################################
//############################### FREE FUNCTIONS ############################
//###########################################################################

void mesh_core::appendPoints(
    EigenSTL::vector_Vector3d& vector,
    int npoints,
    const double *data)
{
  int base = vector.size();
  vector.resize(base + npoints);
  for (int i = 0; i < npoints ; ++i)
  {
    vector[base+i] = Eigen::Vector3d(data[i*3+0],
                                     data[i*3+1],
                                     data[i*3+2]);
  }
}

void mesh_core::appendPointsTransformed(
    EigenSTL::vector_Vector3d& vector,
    const Eigen::Affine3d& xform,
    int npoints,
    const double *data)
{
  int base = vector.size();
  vector.resize(base + npoints);
  for (int i = 0; i < npoints ; ++i)
  {
    vector[base+i] = xform * Eigen::Vector3d(
                                     data[i*3+0],
                                     data[i*3+1],
                                     data[i*3+2]);
  }
}

//###########################################################################
//############################### Plane #####################################
//###########################################################################


mesh_core::Plane::Plane(
      const Eigen::Vector3d& normal,
      double d)
  : normal_(normal.normalized())
  , d_(d)
{ }

mesh_core::Plane::Plane(
      const Eigen::Vector3d& normal,
      const Eigen::Vector3d& point_on_plane)
  : normal_(normal.normalized())
{
  d_ = -point_on_plane.dot(normal_);
}

mesh_core::Plane::Plane(
      const EigenSTL::vector_Vector3d& points)
{
  if (points.size() <= 3)
    from3Points(points);
  else
    leastSquaresGeneral(points);
}

void mesh_core::Plane::from3Points(
      const EigenSTL::vector_Vector3d& points)
{
  Eigen::Vector3d ab, ac, norm;
  int npoints = points.size();
  if (npoints > 2)
  {
    ab = points[1] - points[0];
    ac = points[2] - points[0];
    norm = ab.cross(ac);
  }
  else if (npoints == 2)
  {
    ab = points[1] - points[0];
    ac(0) = ab(1);
    ac(1) = ab(2);
    ac(2) = ab(0);
    norm = ab.cross(ac);
  }
  else if (npoints == 1)
  {
    *this = Plane(Eigen::Vector3d(0,0,1), points[0]);
    return;
  }
  else
  {
    *this = Plane();
    return;
  }

  if (norm.squaredNorm() <= std::numeric_limits<double>::epsilon())
  {
    ac(0) = ab(1);
    ac(1) = ab(2);
    ac(2) = ab(0);
    norm = ab.cross(ac);
    if (norm.squaredNorm() <= std::numeric_limits<double>::epsilon())
    {
      norm = Eigen::Vector3d(0,0,1);
    }
  }

  *this = Plane(norm, points[0]);
}

void mesh_core::Plane::leastSquaresFast(
      const EigenSTL::vector_Vector3d& points,
      Eigen::Vector3d* average)
{
  Eigen::Matrix3d m;
  Eigen::Vector3d b;
  Eigen::Vector3d c;

  m.setZero();
  b.setZero();
  c.setZero();
  
  EigenSTL::vector_Vector3d::const_iterator p = points.begin();
  EigenSTL::vector_Vector3d::const_iterator end = points.end();
  for ( ; p != end ; ++p)
  {
    m(0,0) += p->x() * p->x();
    m(1,0) += p->x() * p->y();
    m(2,0) += p->x();
    m(1,1) += p->y() * p->y();
    m(2,1) += p->y();
    b(0) += p->x() * p->z();
    b(1) += p->y() * p->z();
    b(2) += p->z();
    c += *p;
  }
  m(0,1) = m(1,0);
  m(0,2) = m(2,0);
  m(1,2) = m(2,1);
  m(2,2) = double(points.size());
  c *= 1.0/double(points.size());

  normal_ = m.colPivHouseholderQr().solve(b);
  if (normal_.squaredNorm() > std::numeric_limits<double>::epsilon())
    normal_.normalize();
  
  d_ = -c.dot(normal_);

  if (average)
    *average = c;
}

void mesh_core::Plane::leastSquaresGeneral(
      const EigenSTL::vector_Vector3d& points,
      Eigen::Vector3d* average)
{
  if (points.empty())
  {
    normal_ = Eigen::Vector3d(0,0,1);
    d_ = 0;
    if (average)
      *average = Eigen::Vector3d::Zero();
    return;
  }

  // find c, the average of the points
  Eigen::Vector3d c;
  c.setZero();

  EigenSTL::vector_Vector3d::const_iterator p = points.begin();
  EigenSTL::vector_Vector3d::const_iterator end = points.end();
  for ( ; p != end ; ++p)
    c += *p;

  c *= 1.0/double(points.size());

  // Find the matrix
  Eigen::Matrix3d m;
  m.setZero();

  p = points.begin();
  for ( ; p != end ; ++p)
  {
    Eigen::Vector3d cp = *p - c;
    m(0,0) += cp.x() * cp.x();
    m(1,0) += cp.x() * cp.y();
    m(2,0) += cp.x() * cp.z();
    m(1,1) += cp.y() * cp.y();
    m(2,1) += cp.y() * cp.z();
    m(2,2) += cp.z() * cp.z();
  }
  m(0,1) = m(1,0);
  m(0,2) = m(2,0);
  m(1,2) = m(2,1);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(m);
  if (eigensolver.info() == Eigen::Success)
  {
    normal_ = eigensolver.eigenvectors().col(0);
    normal_.normalize();
  }
  else
  {
    normal_ = Eigen::Vector3d(0,0,1);
  }

  d_ = -c.dot(normal_);

  if (average)
    *average = c;
}

//###########################################################################
//############################### PlaneProjection ###########################
//###########################################################################


mesh_core::PlaneProjection::PlaneProjection(
      const Eigen::Vector3d& normal,
      const Eigen::Vector3d& origin,
      const Eigen::Vector3d& x_axis)
  : Plane(normal, origin)
  , origin_(origin)
{
  initMatrix(x_axis);
}

mesh_core::PlaneProjection::PlaneProjection(
      const Plane& plane,
      const Eigen::Vector3d& point_near_origin,
      const Eigen::Vector3d& x_axis)
  : Plane(plane)
{
  double dist = point_near_origin.dot(normal_) + d_;
  origin_ = point_near_origin - dist * normal_;
  initMatrix(x_axis);
}

mesh_core::PlaneProjection::PlaneProjection(
      const EigenSTL::vector_Vector3d& points,
      const Eigen::Vector3d* point_near_origin,
      const Eigen::Vector3d& x_axis)
  : Plane()
{
  Eigen::Vector3d near_origin;
  leastSquaresGeneral(points, &near_origin);
  if (point_near_origin)
    near_origin = *point_near_origin;

  double dist = near_origin.dot(normal_) + d_;
  origin_ = near_origin - dist * normal_;
  initMatrix(x_axis);
}

void mesh_core::PlaneProjection::initMatrix(
      const Eigen::Vector3d& x_axis)
{
  y_axis_ = normal_.cross(x_axis);
  if (y_axis_.squaredNorm() < std::numeric_limits<double>::epsilon() * 100.0)
  {
    Eigen::Vector3d x_axis2(normal_.y(), normal_.z(), normal_.x());
  }
  y_axis_.normalize();
  x_axis_ = y_axis_.cross(normal_);
}


Eigen::Vector2d mesh_core::PlaneProjection::project(const Eigen::Vector3d& p) const
{
  Eigen::Vector3d ptrans(p - origin_);
  return Eigen::Vector2d(ptrans.dot(x_axis_), ptrans.dot(y_axis_));
}

Eigen::Vector3d mesh_core::PlaneProjection::extract(
      const Eigen::Vector2d& p) const
{
  return origin_ + p.x() * x_axis_ + p.y() * y_axis_;
}

void mesh_core::PlaneProjection::getFrame(Eigen::Affine3d& frame) const
{
  frame.setIdentity();
  frame.translation() = origin_;
  frame.linear().col(0) = x_axis_;
  frame.linear().col(1) = y_axis_;
  frame.linear().col(2) = normal_;
}

Eigen::Quaterniond mesh_core::PlaneProjection::getOrientation() const
{
  Eigen::Matrix3d m;
  m.col(0) = x_axis_;
  m.col(1) = y_axis_;
  m.col(2) = normal_;
  return Eigen::Quaterniond(m);
}


//###########################################################################
//############################### LineSegment2D #############################
//###########################################################################

void mesh_core::LineSegment2D::initialize(
      const Eigen::Vector2d& a,
      const Eigen::Vector2d& b)
{
  pt_[0] = a;
  pt_[1] = b;
  Eigen::Vector2d delta = b - a;

  if (std::abs(delta.x()) <= std::numeric_limits<double>::epsilon())
  {
    vertical_ = true;
    if (std::abs(delta.y()) <= std::numeric_limits<double>::epsilon())
      inv_dx_ = 0.0;
    else
      inv_dx_ = 1.0 / delta.y();
  }
  else
  {
    vertical_ = false;
    inv_dx_ = 1.0 / delta.x();
    slope_ = delta.y() * inv_dx_;
    y_intercept_ = a.y() - slope_ * a.x();
  }
}

mesh_core::LineSegment2D::LineSegment2D(
      const Eigen::Vector2d& a,
      const Eigen::Vector2d& b)
{
  initialize(a,b);
}

bool mesh_core::LineSegment2D::intersect(
      const LineSegment2D& other,
      Eigen::Vector2d& intersection,
      bool& parallel) const
{
  const LineSegment2D& a = *this;
  const LineSegment2D& b = other;

  if (a.vertical_)
  {
    if (b.vertical_)
    {
      parallel = true;
      intersection = a.pt_[0];
      if (a.pt_[0].x() != b.pt_[0].x())
        return false;

      double aymin = std::min(a.pt_[0].y(), a.pt_[1].y());
      double aymax = std::max(a.pt_[0].y(), a.pt_[1].y());
      double bymin = std::min(b.pt_[0].y(), b.pt_[1].y());
      double bymax = std::max(b.pt_[0].y(), b.pt_[1].y());

      if (bymax < aymin)
        return false;
      if (bymin > aymax)
        return false;

      if (bymax <= aymax)
        intersection.y() = bymax;
      else if (bymin >= aymin)
        intersection.y() = bymin;
      else
        intersection.y() = aymin;

      return true;
    }
    parallel = false;
    intersection.x() = a.pt_[0].x();
    intersection.y() = b.slope_ * intersection.x() + b.y_intercept_;

    double tb = (intersection.x() - b.pt_[0].x()) * b.inv_dx_;
    if (tb > 1.0 || tb < 0.0)
      return false;
    
    if (b.inv_dx_ == 0.0)
    {
      if (intersection.y() != a.pt_[0].y())
        return false;
    }
    else
    {
      double ta = (intersection.y() - a.pt_[0].y()) * b.inv_dx_;
      if (ta > 1.0 || ta < 0.0)
        return false;
    }
    
    return true;
  }
  else if (b.vertical_)
  {
    return b.intersect(a, intersection, parallel);
  }
  else
  {
    double bottom = a.slope_ - b.slope_;
    if (std::abs(bottom) < std::numeric_limits<double>::epsilon())
    {
      parallel = true;
      intersection.setZero();
      return false;
    }

    parallel = false;
    intersection.x() = (b.y_intercept_ - a.y_intercept_) / bottom;
    intersection.y() = a.slope_ * intersection.x() + a.y_intercept_;

    double ta = (intersection.x() - a.pt_[0].x()) * b.inv_dx_;
    if (ta > 1.0 || ta < 0.0)
      return false;
    
    double tb = (intersection.x() - b.pt_[0].x()) * b.inv_dx_;
    if (tb > 1.0 || tb < 0.0)
      return false;
    
    return true;
  }
}

#if 1

bool acorn_closest_debug = false;

// return closest point on line segment to the given point, and the distance
// betweeen them.
double mesh_core::closestPointOnLine(
      const Eigen::Vector3d& line_a,
      const Eigen::Vector3d& line_b,
      const Eigen::Vector3d& point,
      Eigen::Vector3d& closest_point)
{
  Eigen::Vector3d ab = line_b - line_a;
  Eigen::Vector3d ab_norm = ab.normalized();
  Eigen::Vector3d ap = point - line_a;

  Eigen::Vector3d closest_point_rel = ab_norm.dot(ap) * ab_norm;

  double dp = ab.dot(closest_point_rel);
  if (dp < 0.0)
  {
    closest_point = line_a;
  }
  else if (dp > ab.squaredNorm())
  {
    closest_point = line_b;
  }
  else
  {
    closest_point = line_a + closest_point_rel;
  }

  return (closest_point - point).norm();
}

// return closest point on triangle to the given point, and the distance
// betweeen them.
// If distance is greater than max_dist then skip the calculations and just return the approximate distance (anything greater than max_dist).
double mesh_core::closestPointOnTriangle(
      const Eigen::Vector3d& tri_a,
      const Eigen::Vector3d& tri_b,
      const Eigen::Vector3d& tri_c,
      const Eigen::Vector3d& point,
      Eigen::Vector3d& closest_point,
      double max_dist)
{
  Eigen::Vector3d ab = tri_b - tri_a;
  Eigen::Vector3d ac = tri_c - tri_a;
  Eigen::Vector3d n = ab.cross(ac);
  Eigen::Vector3d norm = n.normalized();

  Eigen::Vector3d ap = point - tri_a;
  double dist = norm.dot(ap);
  double abs_dist = std::abs(dist);
  
  if (abs_dist >= max_dist)
    return abs_dist;

  closest_point = point - dist * norm;

if (acorn_closest_debug)
{
  logInform(" CDB: tri_a     (%8.4f %8.4f %8.4f)",
    tri_a.x(),
    tri_a.y(),
    tri_a.z());
  logInform(" CDB: tri_b     (%8.4f %8.4f %8.4f)",
    tri_b.x(),
    tri_b.y(),
    tri_b.z());
  logInform(" CDB: tri_c     (%8.4f %8.4f %8.4f)",
    tri_c.x(),
    tri_c.y(),
    tri_c.z());
  logInform(" CDB: point     (%8.4f %8.4f %8.4f)",
    point.x(),
    point.y(),
    point.z());
  logInform(" CDB: intersect (%8.4f %8.4f %8.4f)",
    closest_point.x(),
    closest_point.y(),
    closest_point.z());
}

  Eigen::Vector3d ab_norm = ab.cross(norm);
if (acorn_closest_debug)
{
  logInform(" CDB: ab_norm   (%8.4f %8.4f %8.4f) dp=%8.4f >? da=%8.4f",
    ab_norm.x(),
    ab_norm.y(),
    ab_norm.z(),
      ab_norm.dot(point),
      ab_norm.dot(tri_a));
}
  if (ab_norm.dot(point) > ab_norm.dot(tri_a))
  {
if (acorn_closest_debug)
{
  logInform(" CDB: beyond ab");
}
    return closestPointOnLine(tri_a, tri_b, point, closest_point);
  }
  
  Eigen::Vector3d ac_norm = ac.cross(norm);
if (acorn_closest_debug)
{
  logInform(" CDB: ac_norm   (%8.4f %8.4f %8.4f) dp=%8.4f <? da=%8.4f",
    ac_norm.x(),
    ac_norm.y(),
    ac_norm.z(),
      ac_norm.dot(point),
      ac_norm.dot(tri_a));
}
  if (ac_norm.dot(point) < ac_norm.dot(tri_a))
  {
if (acorn_closest_debug)
{
  logInform(" CDB: beyond ac");
}
    return closestPointOnLine(tri_a, tri_c, point, closest_point);
  }
  
  Eigen::Vector3d bc = tri_c - tri_b;
  Eigen::Vector3d bc_norm = bc.cross(norm);
if (acorn_closest_debug)
{
  logInform(" CDB: ab_norm   (%8.4f %8.4f %8.4f) dp=%8.4f >? db=%8.4f",
    bc_norm.x(),
    bc_norm.y(),
    bc_norm.z(),
      bc_norm.dot(point),
      bc_norm.dot(tri_b));
}
  if (bc_norm.dot(point) > bc_norm.dot(tri_b))
  {
if (acorn_closest_debug)
{
  logInform(" CDB: beyond bc");
}
    return closestPointOnLine(tri_b, tri_c, point, closest_point);
  }
  
  return abs_dist;
}

#endif


