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

static void findSphereTouching3PointsColinear(
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
      robot_sphere_representation::findSphereTouching2Points(center,
                                                             radius,
                                                             a,
                                                             b);
    else
      robot_sphere_representation::findSphereTouching2Points(center,
                                                             radius,
                                                             b,
                                                             c);
  }
  else
  {
    if (ac_lensq > bc_lensq)
      robot_sphere_representation::findSphereTouching2Points(center,
                                                             radius,
                                                             a,
                                                             c);
    else
      robot_sphere_representation::findSphereTouching2Points(center,
                                                             radius,
                                                             b,
                                                             c);
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
  double ac_lensq = ac.squaredNorm();
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

  Eigen::Vector3d rel_center = (ac_lensq * n.cross(ab) +
                                ab_lensq * ac.cross(n)) /
                               (2.0 * n_lensq);
  radius = rel_center.norm();
  center = rel_center + a;
}

static void findSphereTouching4PointsCoplanar(
      Eigen::Vector3d& center,
      double& radius,
      const Eigen::Vector3d& a,
      const Eigen::Vector3d& b,
      const Eigen::Vector3d& c,
      const Eigen::Vector3d& d)
{
  robot_sphere_representation::findSphereTouching3Points(center,
                                                         radius,
                                                         a,
                                                         b,
                                                         c);
  if ((center - d).squaredNorm() <= radius)
    return;
  robot_sphere_representation::findSphereTouching3Points(center,
                                                         radius,
                                                         a,
                                                         b,
                                                         d);
  if ((center - c).squaredNorm() <= radius)
    return;
  robot_sphere_representation::findSphereTouching3Points(center,
                                                         radius,
                                                         a,
                                                         c,
                                                         d);
  if ((center - b).squaredNorm() <= radius)
    return;
  robot_sphere_representation::findSphereTouching3Points(center,
                                                         radius,
                                                         b,
                                                         c,
                                                         d);
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
  Eigen::Vector3d rel_center = (ad_lensq * m.col(0).cross(m.col(1)) +
                                ac_lensq * m.col(2).cross(m.col(0)) +
                                ab_lensq * m.col(1).cross(m.col(2))) /
                               (2.0 * det);

  radius = rel_center.norm();
  center = rel_center + a;
}

namespace
{
// calculate sphere bounding points using Emo Welzl's move-to-front algorithm.
struct SphereInfo
{
  SphereInfo(const EigenSTL::vector_Vector3d& points);
  void findSphere(int npoints, int nbound);

  std::vector<const Eigen::Vector3d*> list_;
  Eigen::Vector3d center_;
  double radius_;
  double radius_sq_;
};
}

SphereInfo::SphereInfo(
      const EigenSTL::vector_Vector3d& points)
  : center_(0,0,0)
  , radius_(0)
  , radius_sq_(0)
{
  int npoints = points.size();
  list_.resize(npoints);
  for (int i = 0 ; i < npoints ; i++)
    list_[i] = &points[i];
}


// find the tightest bounding sphere.
// Recursive.
// The first nbound points in list_ are on the boundary.  The rest are inside
// (or need to be checked).
void SphereInfo::findSphere(
      int npoints,
      int nbound)
{
logInform("findSphere %d %d  - %d",nbound,npoints,__LINE__);
  // increase radius by this much to avoid flipping between points that are
  // equal distance from center.
  static const double radius_expand =
                            std::numeric_limits<float>::epsilon() * 1000.0;

logInform("findSphere %d %d  - %d",nbound,npoints,__LINE__);
  switch(nbound)
  {
  case 0:
logInform("findSphere %d %d  - %d",nbound,npoints,__LINE__);
    center_ = Eigen::Vector3d::Zero();
    radius_ = 0;
logInform("findSphere %d %d  - %d",nbound,npoints,__LINE__);
    break;
  case 1:
logInform("findSphere %d %d  - %d",nbound,npoints,__LINE__);
    center_ = *list_[0];
    radius_ = radius_expand;
logInform("findSphere %d %d  - %d",nbound,npoints,__LINE__);
    break;
  case 2:
logInform("findSphere %d %d  - %d",nbound,npoints,__LINE__);
    robot_sphere_representation::findSphereTouching2Points(
            center_,
            radius_,
            *list_[0],
            *list_[1]);
    radius_ += radius_expand;
logInform("findSphere %d %d  - %d",nbound,npoints,__LINE__);
    break;
  case 3:
logInform("findSphere %d %d  - %d",nbound,npoints,__LINE__);
    robot_sphere_representation::findSphereTouching3Points(
            center_,
            radius_,
            *list_[0],
            *list_[1],
            *list_[2]);
    radius_ += radius_expand;
logInform("findSphere %d %d  - %d",nbound,npoints,__LINE__);
    break;
  default:
logInform("findSphere %d %d  - %d",nbound,npoints,__LINE__);
    logError("Bad nbound=%d for findSphere",nbound);
  case 4:
logInform("findSphere %d %d  - %d",nbound,npoints,__LINE__);
    robot_sphere_representation::findSphereTouching4Points(
            center_,
            radius_,
            *list_[0],
            *list_[1],
            *list_[2],
            *list_[3]);
logInform("findSphere %d %d  - %d",nbound,npoints,__LINE__);
    return;
  }
  radius_sq_ = radius_ * radius_;

logInform("findSphere %d %d  - %d",nbound,npoints,__LINE__);
  for (int i = nbound ; i < npoints ; ++i)
  {
logInform("findSphere %d %d  - %d  i=%d  ",nbound,npoints,__LINE__, i);
logInform("r=%f rsq=%f",radius_,radius_sq_);
logInform("center=%f %f %f",center_.x(), center_.y(), center_.z());
logInform("list[i=%d]=0x%08lx",i,(long)list_[i]);
logInform("list[i=%d]=%f %f %f",i,list_[i]->x(), list_[i]->y(), list_[i]->z());
    if ((*list_[i] - center_).squaredNorm() > radius_sq_)
    {
logInform("findSphere %d %d  - %d",nbound,npoints,__LINE__);
      // entry i is a troublemaker, so move it to head of list (i.e. to
      // list_[nbound])
      const Eigen::Vector3d *p = list_[i];
logInform("%.*s move point %d = %f %f %f to head  (nbound=%d npoint=%d)",nbound,"+++++++++",i,p->x(), p->y(), p->z(),nbound,npoints);
      memmove(&list_[nbound+1],
              &list_[nbound],
              sizeof(const Eigen::Vector3d*) * (i - nbound));
logInform("findSphere %d %d  - %d",nbound,npoints,__LINE__);
      list_[nbound] = p;
logInform("findSphere %d %d  - %d",nbound,npoints,__LINE__);

for (int q=0;q<list_.size();q++)
{
logInform("      list[%3d]=%08lx = %f %f %f",
q,list_[q], 
list_[q]->x(),
list_[q]->y(),
list_[q]->z());
}

      // find a sphere that fits the new set of boundary points.
      findSphere(i, nbound + 1);
logInform("findSphere %d %d  - %d",nbound,npoints,__LINE__);
    }
  }
}

void robot_sphere_representation::generateBoundingSphere(
      const EigenSTL::vector_Vector3d& points,
      Eigen::Vector3d& center,
      double &radius)
{
  SphereInfo info(points);

  info.findSphere(points.size(), 0);

  center = info.center_;
  radius = info.radius_;
}
