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

#include <mesh_core/bounding_sphere.h>
#include <console_bridge/console.h>
#include <Eigen/LU>

#if 0
static bool g_verbose = true;
#else
#define g_verbose (false)
#endif

void mesh_core::findSphereTouching2Points(
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
      mesh_core::findSphereTouching2Points(center, radius, a, b);
    else
      mesh_core::findSphereTouching2Points(center, radius, b, c);
  }
  else
  {
    if (ac_lensq > bc_lensq)
      mesh_core::findSphereTouching2Points(center, radius, a, c);
    else
      mesh_core::findSphereTouching2Points(center, radius, b, c);
  }
}

void mesh_core::findSphereTouching3Points(
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
    if (g_verbose)
    {
      logInform("findSphereTouching3Points() COLINEAR QQQQ");
      logInform("           a = (%7.3f %7.3f %7.3f)",
        a.x(),
        a.y(),
        a.z());
      logInform("           b = (%7.3f %7.3f %7.3f)",
        b.x(),
        b.y(),
        b.z());
      logInform("           c = (%7.3f %7.3f %7.3f)",
        c.x(),
        c.y(),
        c.z());
      logInform("           s = (%7.3f %7.3f %7.3f) r=%7.3f",
        center.x(),
        center.y(),
        center.z(),
        radius);
    }
    return;
  }

  Eigen::Vector3d rel_center = (ac_lensq * n.cross(ab) +
                                ab_lensq * ac.cross(n)) /
                               (2.0 * n_lensq);
  radius = rel_center.norm();
  center = rel_center + a;

  if (g_verbose)
  {
    logInform("findSphereTouching3Points()");
    logInform("           a = (%7.3f %7.3f %7.3f)",
      a.x(),
      a.y(),
      a.z());
    logInform("           b = (%7.3f %7.3f %7.3f)",
      b.x(),
      b.y(),
      b.z());
    logInform("           c = (%7.3f %7.3f %7.3f)",
      c.x(),
      c.y(),
      c.z());
    logInform("           s = (%7.3f %7.3f %7.3f) r=%7.3f",
      center.x(),
      center.y(),
      center.z(),
      radius);
  }
}

static void findSphereTouching4PointsCoplanar(
      Eigen::Vector3d& center,
      double& radius,
      const Eigen::Vector3d& a,
      const Eigen::Vector3d& b,
      const Eigen::Vector3d& c,
      const Eigen::Vector3d& d)
{
  mesh_core::findSphereTouching3Points(center,
                                                         radius,
                                                         a,
                                                         b,
                                                         c);
  if (g_verbose)
  {
    double dist = (center - d).norm();
    logInform("    findSphereTouching4PointsCoplanar abc center=(%7.3f %7.3f %7.3f) r=%10.6f d.dist=%10.6f (%10.6f)   (%s)",
      center.x(),
      center.y(),
      center.z(),
      radius,
      dist,
      radius - dist,
      radius - dist < 0 ? "OUTSIDE" : "ok");
  }
  if ((center - d).squaredNorm() <= radius*radius)
    return;
  mesh_core::findSphereTouching3Points(center,
                                                         radius,
                                                         a,
                                                         b,
                                                         d);
  if (g_verbose)
  {
    double dist = (center - c).norm();
    logInform("    findSphereTouching4PointsCoplanar abd center=(%7.3f %7.3f %7.3f) r=%10.6f c.dist=%10.6f (%10.6f)   (%s)",
      center.x(),
      center.y(),
      center.z(),
      radius,
      dist,
      radius - dist,
      radius - dist < 0 ? "OUTSIDE" : "ok");
  }
  if ((center - c).squaredNorm() <= radius*radius)
    return;
  mesh_core::findSphereTouching3Points(center,
                                                         radius,
                                                         a,
                                                         c,
                                                         d);
  if (g_verbose)
  {
    double dist = (center - b).norm();
    logInform("    findSphereTouching4PointsCoplanar acd center=(%7.3f %7.3f %7.3f) r=%10.6f b.dist=%10.6f (%10.6f)   (%s)",
      center.x(),
      center.y(),
      center.z(),
      radius,
      dist,
      radius - dist,
      radius - dist < 0 ? "OUTSIDE" : "ok");
  }
  if ((center - b).squaredNorm() <= radius*radius)
    return;
  mesh_core::findSphereTouching3Points(center,
                                                         radius,
                                                         b,
                                                         c,
                                                         d);
  if (g_verbose)
  {
    double dist = (center - a).norm();
    logInform("    findSphereTouching4PointsCoplanar bcd center=(%7.3f %7.3f %7.3f) r=%10.6f a.dist=%10.6f (%10.6f)   (%s)",
      center.x(),
      center.y(),
      center.z(),
      radius,
      dist,
      radius - dist,
      radius - dist < 0 ? "OUTSIDE" : "ok");
  }
}

void mesh_core::findSphereTouching4Points(
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
  if (std::abs(det) <= std::numeric_limits<double>::epsilon())
  {
    findSphereTouching4PointsCoplanar(center, radius, a,b,c,d);
    if (g_verbose)
    {
      logInform("findSphereTouching4Points() COPLANAR  QQQQ");
      logInform("           a = (%7.3f %7.3f %7.3f)",
        a.x(),
        a.y(),
        a.z());
      logInform("           b = (%7.3f %7.3f %7.3f)",
        b.x(),
        b.y(),
        b.z());
      logInform("           c = (%7.3f %7.3f %7.3f)",
        c.x(),
        c.y(),
        c.z());
      logInform("           d = (%7.3f %7.3f %7.3f)",
        d.x(),
        d.y(),
        d.z());
      logInform("           s = (%7.3f %7.3f %7.3f) r=%7.3f",
        center.x(),
        center.y(),
        center.z(),
        radius);
    }
    return;
  }
  
  double ab_lensq = m.col(0).squaredNorm();
  double ac_lensq = m.col(1).squaredNorm();
  double ad_lensq = m.col(2).squaredNorm();
  Eigen::Vector3d rel_center = ((ad_lensq * m.col(0).cross(m.col(1))) +
                                (ac_lensq * m.col(2).cross(m.col(0))) +
                                (ab_lensq * m.col(1).cross(m.col(2)))) /
                               (2.0 * det);

  radius = rel_center.norm();
  center = rel_center + a;

  if (g_verbose)
  {
    logInform("findSphereTouching4Points()");
    logInform("           a = (%7.3f %7.3f %7.3f)",
      a.x(),
      a.y(),
      a.z());
    logInform("           b = (%7.3f %7.3f %7.3f)",
      b.x(),
      b.y(),
      b.z());
    logInform("           c = (%7.3f %7.3f %7.3f)",
      c.x(),
      c.y(),
      c.z());
    logInform("           d = (%7.3f %7.3f %7.3f)",
      d.x(),
      d.y(),
      d.z());
    logInform("           s = (%7.3f %7.3f %7.3f) r=%7.3f",
      center.x(),
      center.y(),
      center.z(),
      radius);
  }
}

namespace
{
  // calculate sphere bounding points using Emo Welzl's move-to-front algorithm.
  struct SphereInfo
  {
    SphereInfo(const EigenSTL::vector_Vector3d& points);
    void findSphere(int nbound, int npoints);
    void findStartingPoints();

    const EigenSTL::vector_Vector3d& points_;
    std::vector<const Eigen::Vector3d*> list_;
    Eigen::Vector3d center_;
    double radius_;
    double radius_sq_;
  };
}

SphereInfo::SphereInfo(
      const EigenSTL::vector_Vector3d& points)
  : points_(points)
  , center_(0,0,0)
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
// The first nbound points in list_ are known to be on the boundary.  The rest need to be checked.
void SphereInfo::findSphere(
      int nbound,
      int npoints)
{
  // increase radius by this much to avoid flipping between points that are
  // equal distance from center.
  static const double radius_expand =
                            std::numeric_limits<float>::epsilon() * 1000.0;

  if (g_verbose)
  {
    logInform(" findSphere(%d,%3d) ENTER",
      nbound,npoints);
  }

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
    mesh_core::findSphereTouching2Points(
            center_,
            radius_,
            *list_[0],
            *list_[1]);
    radius_ += radius_expand;
    break;
  case 3:
    mesh_core::findSphereTouching3Points(
            center_,
            radius_,
            *list_[0],
            *list_[1],
            *list_[2]);
    radius_ += radius_expand;
    break;
  default:
    logError("Bad nbound=%d for findSphere",nbound);
  case 4:
    mesh_core::findSphereTouching4Points(
            center_,
            radius_,
            *list_[0],
            *list_[1],
            *list_[2],
            *list_[3]);
    radius_ += radius_expand;
    radius_sq_ = radius_ * radius_;
    if (g_verbose)
    {
      logInform(" findSphere(%d,%4d) begin check   s = (%7.3f %7.3f %7.3f) r=%10.6f   rsq=%10.6f    ",
        nbound,
        npoints,
        center_.x(),
        center_.y(),
        center_.z(),
        radius_,
        radius_sq_);
      for (int i=0; i<nbound; ++i)
      {
        double d = (*list_[i] - center_).norm();
        logInform("    BOUND: list_[%4d] = %4d         (%7.3f %7.3f %7.3f) d=%10.6f (%10.6f)    (BOUND)    ",
          i,
          int(list_[i] - &points_[0]),
          list_[i]->x(),
          list_[i]->y(),
          list_[i]->z(),
          d,
          radius_ - d);
      }
    }
    return;
  }
  radius_sq_ = radius_ * radius_;

  if (g_verbose)
  {
    logInform(" findSphere(%d,%4d) begin check   s = (%7.3f %7.3f %7.3f) r=%10.6f   rsq=%10.6f    ",
      nbound,
      npoints,
      center_.x(),
      center_.y(),
      center_.z(),
      radius_,
      radius_sq_);
    for (int i=0; i<nbound; ++i)
    {
      double d = (*list_[i] - center_).norm();
      logInform("    BOUND: list_[%4d] = %4d         (%7.3f %7.3f %7.3f) d=%10.6f (%10.6f)    (BOUND)    ",
        i,
        int(list_[i] - &points_[0]),
        list_[i]->x(),
        list_[i]->y(),
        list_[i]->z(),
        d,
        radius_ - d);
    }
  }

  for (int i = nbound ; i < npoints ; ++i)
  {

    if (g_verbose)
    {
      double d = (*list_[i] - center_).norm();
      logInform("    check  list_[%4d] = %4d         (%7.3f %7.3f %7.3f) d=%10.6f (%10.6f)    (%s)    ",
        i,
        int(list_[i] - &points_[0]),
        list_[i]->x(),
        list_[i]->y(),
        list_[i]->z(),
        d,
        radius_ - d,
        ((*list_[i] - center_).squaredNorm() > radius_sq_) ? "OUTSIDE" : "ok");
    }

    if ((*list_[i] - center_).squaredNorm() > radius_sq_)
    {
      // entry i is a troublemaker, so move it to head of list (i.e. to
      // list_[nbound])
      const Eigen::Vector3d *p = list_[i];

      if (g_verbose)
      {
        logInform("        Point %4d outside -- move to list_[%4d]    ",
          int(list_[i] - &points_[0]),
          nbound);
      }

      memmove(&list_[nbound+1],
              &list_[nbound],
              sizeof(const Eigen::Vector3d*) * (i - nbound));
      list_[nbound] = p;

      // find a sphere that has the new boundary point and contains all points that have already been checked so far.
      findSphere(nbound + 1, i + 1);

      if (g_verbose)
      {
        logInform(" findSphere(%d,%4d) cont          s = (%7.3f %7.3f %7.3f) r=%10.6f   rsq=%10.6f    ",
          nbound,
          npoints,
          center_.x(),
          center_.y(),
          center_.z(),
          radius_,
          radius_sq_);
      }
    }
  }

  if (g_verbose)
  {
    logInform(" findSphere(%d,%4d) RETURN        s = (%7.3f %7.3f %7.3f) r=%10.6f   rsq=%10.6f    ",
      nbound,
      npoints,
      center_.x(),
      center_.y(),
      center_.z(),
      radius_,
      radius_sq_);
  }
}

// optional, but makes it faster
// places the 6 points at the extremes of the axes at the beginning of the list
void SphereInfo::findStartingPoints()
{
  if (points_.size() < 6)
    return;

  EigenSTL::vector_Vector3d::const_iterator xmin = points_.begin();
  EigenSTL::vector_Vector3d::const_iterator xmax = points_.begin();
  EigenSTL::vector_Vector3d::const_iterator ymin = points_.begin();
  EigenSTL::vector_Vector3d::const_iterator ymax = points_.begin();
  EigenSTL::vector_Vector3d::const_iterator zmin = points_.begin();
  EigenSTL::vector_Vector3d::const_iterator zmax = points_.begin();

  EigenSTL::vector_Vector3d::const_iterator it = points_.begin();
  EigenSTL::vector_Vector3d::const_iterator end = points_.end();

  // find most distant points in each dimension
  for ( ; it != end ; ++it )
  {
    if (xmin->x() > it->x())
      xmin = it;
    if (xmax->x() < it->x())
      xmax = it;
    if (ymin->y() > it->y())
      ymin = it;
    if (ymax->y() < it->y())
      ymax = it;
    if (zmin->z() > it->z())
      zmin = it;
    if (zmax->z() < it->z())
      zmax = it;
  }

  std::swap(list_[0], list_[xmin - points_.begin()]);
  std::swap(list_[1], list_[xmax - points_.begin()]);
  std::swap(list_[2], list_[ymin - points_.begin()]);
  std::swap(list_[3], list_[ymax - points_.begin()]);
  std::swap(list_[4], list_[zmin - points_.begin()]);
  std::swap(list_[5], list_[zmax - points_.begin()]);
}

void mesh_core::generateBoundingSphere(
      const EigenSTL::vector_Vector3d& points,
      Eigen::Vector3d& center,
      double &radius)
{
  SphereInfo info(points);

  info.findStartingPoints();
  info.findSphere(0, points.size());

  center = info.center_;
  radius = info.radius_;
}
