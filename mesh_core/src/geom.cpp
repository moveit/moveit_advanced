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
  leastSquaresFast(points);
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
  if (normal_.squaredNorm() > std::numeric_limits<float>::epsilon())
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
const Eigen::Vector3d& evalues = eigensolver.eigenvalues();
logInform("Eigenvalues: %10.6f %10.6f %10.6f",
evalues.x(),
evalues.y(),
evalues.z());
const Eigen::Matrix3d& evecs = eigensolver.eigenvectors();
for (int i=0;i<3;i++)
{
  logInform("Eigenvec[%d]: %10.6f %10.6f %10.6f",
    evecs(0,i),
    evecs(1,i),
    evecs(2,i));
}
    
    normal_ = eigensolver.eigenvectors().col(0);
    normal_.normalize();
  }
  else
  {
logInform("SelfAdjointEigenSolver failed.  Using normal=0,0,1");
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
  if (y_axis_.squaredNorm() < std::numeric_limits<float>::epsilon() * 100.0)
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

Eigen::Vector3d mesh_core::PlaneProjection::to3D(const Eigen::Vector2d& p) const
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
//############################### LINE2D ####################################
//###########################################################################


