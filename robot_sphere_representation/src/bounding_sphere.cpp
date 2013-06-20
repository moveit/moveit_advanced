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

#include <moveit/robot_sphere_representation/bounding_sphere.h>
#include <console_bridge/console.h>
#include <Eigen/LU>

void robot_sphere_representation::findSphereTouching2Points(
      Eigen::Vector3d& center,
      double& radius,
      const Eigen::Vector3d& a,
      const Eigen::Vector3d& b)
{
  center = (a + b) * 0.5;
  radius = (center - a).norm();
}

static void robot_sphere_representation::findSphereTouching3PointsColinear(
      Eigen::Vector3d& center,
      double& radius,
      const Eigen::Vector3d& a,
      const Eigen::Vector3d& b,
      const Eigen::Vector3d& c,
      double ab_lensq,
      double bc_lensq,
      double ac_lensq)
{
  if (ab_lensq > ac_lensq)
  {
    if (ab_lensq > bc_lensq)
      findSphereTouching2Points(center, radius, a, b);
    else
      findSphereTouching2Points(center, radius, b, c);
  }
  else
  {
    if (ac_lensq > bc_lensq)
      findSphereTouching2Points(center, radius, a, c);
    else
      findSphereTouching2Points(center, radius, b, c);
  }
}

void robot_sphere_representation::findSphereTouching3Points(
      Eigen::Vector3d& center,
      double& radius,
      const Eigen::Vector3d& a,
      const Eigen::Vector3d& b,
      const Eigen::Vector3d& c)
{
  Eigen::Vector3d ab = b - a;
  Eigen::Vector3d ac = c - a;
  Eigen::Vector3d n = ab.cross(ac);

  double ab_lensq = ab.squaredNorm();
  double ac_lensq = ab.squaredNorm();
  double n_lensq = n.squaredNorm();

  // points are colinear?
  if (n_lensq <= std::numeric_limits<double>::epsilon())
  {
    findSphereTouching3PointsColinear(
          center,
          radius,
          a,
          b,
          c,
          ab_lensq,
          (c - b).squaredNorm(),
          ac_lensq);
    return;
  }

  Eigen::Vector3d c = ((bc_lensq * n.cross(ab) + ab_lensq * bc.cross(n)) / (2.0 * n_lensq));
  radius = c.normSquared();
  center = c + a;
}

static void robot_sphere_representation::findSphereTouching4PointsCoplanar(
      Eigen::Vector3d& center,
      double& radius,
      const Eigen::Vector3d& a,
      const Eigen::Vector3d& b,
      const Eigen::Vector3d& c,
      const Eigen::Vector3d& d)
{
  findSphereTouching3Points(center, radius, a, b, c);
  if ((center - d).squaredNorm() <= radius)
    return;
  findSphereTouching3Points(center, radius, a, b, d);
  if ((center - c).squaredNorm() <= radius)
    return;
  findSphereTouching3Points(center, radius, a, c, d);
  if ((center - b).squaredNorm() <= radius)
    return;
  findSphereTouching3Points(center, radius, b, c, d);
}

void robot_sphere_representation::findSphereTouching4Points(
      Eigen::Vector3d& center,
      double& radius,
      const Eigen::Vector3d& a,
      const Eigen::Vector3d& b,
      const Eigen::Vector3d& c,
      const Eigen::Vector3d& d)
{
  Eigen::Matrix3d m;
  m.col(0) = b - a;
  m.col(1) = c - a;
  m.col(2) = d - a;

  double det = m.determinant();

  // points are coplanar?
  if (det <= std::numeric_limits<double>::epsilon())
  {
    findSphereTouching4PointsCoplanar(center, radius, a,b,c,d);
    return;
  }
  
  double ab_lensq = m.col(0).squaredNorm();
  double ac_lensq = m.col(1).squaredNorm();
  double ad_lensq = m.col(2).squaredNorm();
  Eigen::Vector3d c = (ad_lensq * m.col(0).cross(m.col(1)) +
                       ac_lensq * m.col(2).cross(m.col(0)) +
                       ab_lensq * m.col(1).cross(m.col(2))) / (2.0 * det);

  radius = c.normSquared();
  center = c + a;
}

namespace
{
// calculate sphere bounding points using Emo Welzl's move-to-front algorithm.
struct SphereInfo
{
  SphereInfo(const EigenSTL::vector_Vector3d& points);
  findSphere(int npoints, int nbound);

  std::vector<Eigen::Vector3d*> list_;
  Eigen::Vector3d center_;
  double radius_;
};
}

void SphereInfo::SphereInfo(
      const EigenSTL::vector_Vector3d& points)
  : center_(0,0,0)
  , radius_(0)
{
  int npoints = points.size();
  list_.resize(npoints);
  for (int i = 0 ; i < npoints ; i++)
    list_[i] = &points[i];
}


// find the tightest bounding sphere.
// Recursive.
// The first nbound points in list_ are on the boundary.  The rest are inside (or need to be checked).
void SphereInfo::findSphere(
      int npoints,
      int nbound)
{
  // increase radius by this much to avoid flipping between points that are
  // equal distance from center.
  static const double radius_expand = std::numeric_limits<float>::epsilon() * 1000.0;

  switch(nbound)
  {
  case 0:
    center_ = Eigen::Vector3d::Zero();
    radius_ = 0;
    break;
  case 1:
    center_ = *list_[0];
    radius_ = radius_expand;
    break;
  case 2:
    void robot_sphere_representation::findSphereTouching2Points(
            center_,
            radius_,
            *list_[0],
            *list_[1]);
    radius_ += radius_expand;
    break;
  case 3:
    void robot_sphere_representation::findSphereTouching3Points(
            center_,
            radius_,
            *list[0],
            *list[1],
            *list[2]);
    radius_ += radius_expand;
    break;
  default:
    logError("Bad nbound=%d for findSphere",nbound);
  case 4:
    void robot_sphere_representation::findSphereTouching4Points(
            center_,
            radius_,
            *list[0],
            *list[1],
            *list[2],
            *list[3]);
    return;
  }

  for (int i = nbound ; i < npoints ; ++i)
  {
    if ((*list[i] - center_) > radius_)
    {
      // entry i is a troublemaker, so move it to head of list (i.e. to list_[nbound])
      Eigen::Vector3d *p = list_[i];
      memmove(&list_[nbound+1], &list_[i], sizeof(Eigen::Vector3d*) * (i - nbound));
      list_[nbound] = p;

      // find a sphere that fits the new set of boundary points.
      findSphere(i, nbound + 1);
    }
  }
}

void robot_sphere_representation::generateBoundingSphere(
      const EigenSTL::vector_Vector3d& points,
      Eigen::Vector3d& center,
      double &radius)
{
  SphereInfo info(points);

  info.findSphere(&info.list_[0], points.size(), 0);

  center = info.center;
  radius = info.radius;
}
