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

int xx_icnt;
int xx_scnt0;
int xx_scnt1;
int xx_scnt2;
int xx_scnt3;
int xx_scnt4;

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

  // DEBUG STUFF
  const Eigen::Vector3d* base_point_;
  Eigen::Vector3d* last_pt_;
  int iteration_max_;
  int iteration_;
  EigenSTL::vector_Vector3d * accounted_;
  int *used_mask_;
};
}

SphereInfo::SphereInfo(
      const EigenSTL::vector_Vector3d& points)
  : center_(0,0,0)
  , radius_(0)
  , radius_sq_(0)
  , last_pt_(NULL)
  , iteration_max_(-1)
  , iteration_(0)
  , accounted_(NULL)
  , used_mask_(NULL)
{
  int npoints = points.size();
  list_.resize(npoints);
  for (int i = 0 ; i < npoints ; i++)
    list_[i] = &points[i];

base_point_ = &points[0];
}


// find the tightest bounding sphere.
// Recursive.
// The first nbound points in list_ are on the boundary.  The rest are inside
// (or need to be checked).
void SphereInfo::findSphere(
      int npoints,
      int nbound)
{
  // increase radius by this much to avoid flipping between points that are
  // equal distance from center.
  static const double radius_expand =
                            std::numeric_limits<float>::epsilon() * 1000.0;

  if (accounted_)
  {
    accounted_->clear();
    for (int i=0; i<nbound; i++)
    {
      accounted_->push_back(*list_[i]);
    }
  }
  if (used_mask_)
    *used_mask_ = (1 << nbound) - 1;

xx_icnt++;
  switch(nbound)
  {
  case 0:
xx_scnt0++;
    center_ = Eigen::Vector3d::Zero();
    radius_ = 0;
logInform("%.*s findsphere(%3d,%d) find sphere 0 point r=%f",nbound,"+++++++++",npoints,nbound,radius_);
    break;
  case 1:
xx_scnt1++;
    center_ = *list_[0];
    radius_ = radius_expand;
logInform("%.*s findsphere(%3d,%d) find sphere 1 point r=%f",nbound,"+++++++++",npoints,nbound,radius_);
logInform("%.*s findsphere(%3d,%d)            %f %f %f",nbound,"+++++++++",npoints,nbound, list_[0]->x(), list_[0]->y(), list_[0]->z());
    break;
  case 2:
xx_scnt2++;
    robot_sphere_representation::findSphereTouching2Points(
            center_,
            radius_,
            *list_[0],
            *list_[1]);
    radius_ += radius_expand;
logInform("%.*s findsphere(%3d,%d) find sphere 2 point r=%f",nbound,"+++++++++",npoints,nbound,radius_);
logInform("%.*s findsphere(%3d,%d)            %f %f %f",nbound,"+++++++++",npoints,nbound, list_[0]->x(), list_[0]->y(), list_[0]->z());
logInform("%.*s findsphere(%3d,%d)            %f %f %f",nbound,"+++++++++",npoints,nbound, list_[1]->x(), list_[1]->y(), list_[1]->z());
    break;
  case 3:
xx_scnt3++;
    robot_sphere_representation::findSphereTouching3Points(
            center_,
            radius_,
            *list_[0],
            *list_[1],
            *list_[2]);
    radius_ += radius_expand;
logInform("%.*s findsphere(%3d,%d) find sphere 3 point r=%f",nbound,"+++++++++",npoints,nbound,radius_);
logInform("%.*s findsphere(%3d,%d)            %f %f %f",nbound,"+++++++++",npoints,nbound, list_[0]->x(), list_[0]->y(), list_[0]->z());
logInform("%.*s findsphere(%3d,%d)            %f %f %f",nbound,"+++++++++",npoints,nbound, list_[1]->x(), list_[1]->y(), list_[1]->z());
logInform("%.*s findsphere(%3d,%d)            %f %f %f",nbound,"+++++++++",npoints,nbound, list_[2]->x(), list_[2]->y(), list_[2]->z());
    break;
  default:
    logError("Bad nbound=%d for findSphere",nbound);
  case 4:
xx_scnt4++;
    robot_sphere_representation::findSphereTouching4Points(
            center_,
            radius_,
            *list_[0],
            *list_[1],
            *list_[2],
            *list_[3]);
logInform("%.*s findsphere(%3d,%d) find sphere 4 point r=%f",nbound,"+++++++++",npoints,nbound,radius_);
logInform("%.*s findsphere(%3d,%d)            %f %f %f",nbound,"+++++++++",npoints,nbound, list_[0]->x(), list_[0]->y(), list_[0]->z());
logInform("%.*s findsphere(%3d,%d)            %f %f %f",nbound,"+++++++++",npoints,nbound, list_[1]->x(), list_[1]->y(), list_[1]->z());
logInform("%.*s findsphere(%3d,%d)            %f %f %f",nbound,"+++++++++",npoints,nbound, list_[2]->x(), list_[2]->y(), list_[2]->z());
logInform("%.*s findsphere(%3d,%d)            %f %f %f",nbound,"+++++++++",npoints,nbound, list_[3]->x(), list_[3]->y(), list_[3]->z());
    return;
  }
  radius_sq_ = radius_ * radius_;

#if 1
  if (iteration_max_ >= 0 && iteration_ >= iteration_max_)
    return;
  ++iteration_;
  if (iteration_max_ >= 0 && iteration_ >= iteration_max_)
  {
#if 1
for (int q=0;q<std::min(int(list_.size()),10);q++)
{
logInform("      list[%3d]=%3d       %08lx = %f %f %f",
q,
int(list_[q] - base_point_),
list_[q], 
list_[q]->x(),
list_[q]->y(),
list_[q]->z());
}
#endif
    logInform("%.*s findsphere(%3d,%d) END after gen sphere",nbound,"+++++++++",npoints,nbound);
    return;
  }
#endif

  for (int i = nbound ; i < npoints ; ++i)
  {


    if (accounted_)
    {
      accounted_->push_back(*list_[i]);
    }

if (last_pt_) *last_pt_ = *list_[i];
int orig_idx = int(list_[i] - base_point_);
    if ((*list_[i] - center_).squaredNorm() > radius_sq_)
    {
      // entry i is a troublemaker, so move it to head of list (i.e. to
      // list_[nbound])
      const Eigen::Vector3d *p = list_[i];

#if 0
logInform("%.*s findsphere(%3d,%d) move list[i=%3d]=%d to head",
nbound,"+++++++++",npoints,nbound,i,int(list_[i] - base_point_));
#endif

      memmove(&list_[nbound+1],
              &list_[nbound],
              sizeof(const Eigen::Vector3d*) * (i - nbound));
      list_[nbound] = p;

#if 0
for (int q=0;q<std::min(int(list_.size()),10);q++)
{
logInform("      list[%3d]=%3d       %08lx = %f %f %f",
q,
int(list_[q] - base_point_),
list_[q], 
list_[q]->x(),
list_[q]->y(),
list_[q]->z());
}
#endif

      // find a sphere that fits the new set of boundary points.
      findSphere(i + 1, nbound + 1);
    }
    if (iteration_max_ >= 0 && iteration_ >= iteration_max_)
      return;
    ++iteration_;
    if (iteration_max_ >= 0 && iteration_ >= iteration_max_)
    {
      
#if 1
for (int q=0;q<std::min(int(list_.size()),10);q++)
{
logInform("      list[%3d]=%3d       %08lx = %f %f %f",
q,
int(list_[q] - base_point_),
list_[q], 
list_[q]->x(),
list_[q]->y(),
list_[q]->z());
}
#endif
      logInform("%.*s findsphere(%3d,%d) END after check point list[%3d]=%3d   nbound=%d",nbound,"+++++++++",npoints,nbound, i, orig_idx, nbound);
      return;
    }
  }
}

namespace robot_sphere_representation
{
void generateBoundingSphere1(
      const EigenSTL::vector_Vector3d& points,
      Eigen::Vector3d& center,
      double &radius)
{
  SphereInfo info(points);

  info.findSphere(points.size(), 0);

  center = info.center_;
  radius = info.radius_;
}

void generateBoundingSphere2(
      const EigenSTL::vector_Vector3d& points,
      Eigen::Vector3d& center,
      double &radius)
{
  SphereInfo info(points);

  srand(int(time(NULL)));

  for (int i = 0; i < points.size() ; i++)
  {
    std::swap(info.list_[i], info.list_[i + (rand() % (points.size()-i))]);
  }

  info.findSphere(points.size(), 0);

  center = info.center_;
  radius = info.radius_;
}

void generateBoundingSphere3(
      const EigenSTL::vector_Vector3d& points,
      Eigen::Vector3d& center,
      double &radius)
{
  SphereInfo info(points);

  if (points.size() > 6)
  {
    int ixmin = 0;
    int ixmax = 0;
    int iymin = 0;
    int iymax = 0;
    int izmin = 0;
    int izmax = 0;

    for (int i = 0; i < points.size() ; i++)
    {
      if (info.list_[ixmin]->x() > info.list_[i]->x())
        ixmin = i;
      if (info.list_[ixmax]->x() < info.list_[i]->x())
        ixmax = i;
      if (info.list_[iymin]->y() > info.list_[i]->y())
        iymin = i;
      if (info.list_[iymax]->y() < info.list_[i]->y())
        iymax = i;
      if (info.list_[izmin]->z() > info.list_[i]->z())
        izmin = i;
      if (info.list_[izmax]->z() < info.list_[i]->z())
        izmax = i;
    }
    std::swap(info.list_[0], info.list_[ixmin]);
    std::swap(info.list_[1], info.list_[ixmax]);
    std::swap(info.list_[2], info.list_[iymin]);
    std::swap(info.list_[3], info.list_[iymax]);
    std::swap(info.list_[4], info.list_[izmin]);
    std::swap(info.list_[5], info.list_[izmax]);
  }

  info.findSphere(points.size(), 0);

  center = info.center_;
  radius = info.radius_;
}

void xx_clr()
{
  xx_icnt=0;
  xx_scnt0=0;
  xx_scnt1=0;
  xx_scnt2=0;
  xx_scnt3=0;
  xx_scnt4=0;
}
void xx_print(const char *str)
{
  logInform("generateBoundingSphere %s  it=%3d   sp=%3d,%3d,%3d,%3d,%3d",
    str,
    xx_icnt,
    xx_scnt0,
    xx_scnt1,
    xx_scnt2,
    xx_scnt3,
    xx_scnt4);
}
}

void robot_sphere_representation::generateBoundingSphereDebug(
      const EigenSTL::vector_Vector3d& points,
      Eigen::Vector3d& center,
      double &radius,
      int iteration,
      int* used_mask,
      EigenSTL::vector_Vector3d* corners,
      EigenSTL::vector_Vector3d* accounted,
      Eigen::Vector3d* last_pt)
{
  SphereInfo info(points);

  info.last_pt_ = last_pt;
  info.iteration_max_ = iteration;
  info.accounted_ = accounted;
  info.used_mask_ = used_mask;

  if (last_pt)
    *last_pt = points.empty() ? Eigen::Vector3d::Zero() : points[0];
  if (accounted)
    accounted->clear();
  if (used_mask)
    *used_mask = 0;

  info.findSphere(points.size(), 0);

  if (corners)
  {
    corners->clear();
    for (int i=0; i<4 && i<points.size() ; ++i)
    {
      corners->push_back(*info.list_[i]);
    }
  }
logInform("ITERATIONS: %d",info.iteration_);

  center = info.center_;
  radius = info.radius_;
}

void robot_sphere_representation::generateBoundingSphere(
      const EigenSTL::vector_Vector3d& points,
      Eigen::Vector3d& center,
      double &radius)
{
  xx_clr();
  generateBoundingSphere1(points,center,radius);
  xx_print("1");
#if 0
  double r1=radius;
  xx_clr();
  generateBoundingSphere2(points,center,radius);
  xx_print("1");
  double r2=radius;
  xx_clr();
  generateBoundingSphere3(points,center,radius);
  xx_print("1");
  logInform("r1: %f",r1);
  logInform("r2: %f",r2);
  logInform("r3: %f",radius);
#endif
}
