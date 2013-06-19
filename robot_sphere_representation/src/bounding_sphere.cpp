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

void robot_sphere_representation::findCircleTouching2Points(
      const Eigen::Vector3d& a,
      const Eigen::Vector3d& b,
      Eigen::Vector3d& center,
      double& radius)
{
  center = (a + b) * 0.5;
  radius = (center - a).norm();
}


// generate the circle that touches first 3 points of boundary_
// normal is the normal to the plane of the circle -- NOT a unit normal.
// If points are colinear is set to true and normal=0,0,0.  Else colinear=false.
void robot_sphere_representation::findCircleTouching3Points(
      const Eigen::Vector3d& a,
      const Eigen::Vector3d& b,
      const Eigen::Vector3d& c,
      Eigen::Vector3d& center,
      double& radius,
      Eigen::Vector3d& normal,
      bool colinear)
{
  Eigen::Vector3d ab = b-a;
  Eigen::Vector3d bc = c-b;
  Eigen::Vector3d n = ab.cross(bc);

  // points are colinear?
  if (n.squaredNorm() <= std::numeric_limits<double>::epsilon())
  {
    Eigen::Vector3d ca = a-c;
    double ab_sq = ab.squaredNorm();
    double bc_sq = bc.squaredNorm();
    double ca_sq = ca.squaredNorm();
    if (bc_sq > ab_sq)
    {
      if (ca_sq > bc_sq)
        findCircleTouching2Points(a, c, center, radius);
      else
        findCircleTouching2Points(b, c, center, radius);
    }
    else if (ca_sq > ab_sq)
      findCircleTouching2Points(a, c, center, radius);
    else
      findCircleTouching2Points(a, b, center, radius);
    
    normal = Eigen::Vector3d::Zero();
    colinear = true;
    return;
  }

  colinear = false;

  // define 2D coordinate system in plane of tri with a at origin and b at 1,0
  normal = n;
  Eigen::Vector3d& x = ab;
  Eigen::Vector3d y = n.cross(x);

  double x0 = x.dot(a);
  double y0 = y.dot(a);
  double z0 = z.dot(a);

  Eigen::Vector2d fa(0.0, 0.0);
  Eigen::Vector2d fb(1.0, 0.0);
  Eigen::Vector2d fc(x.dot(c) - x0, y.dot(c) - y0);

  Eigen::Vector2d fac_mid = fc * 0.5;   // (fc - fa)/2
  Eigen::Vector2d fac_dir(-fc.y(), fc.x());

  // perpendicular bisector of ab is vertical at x=0.5
  // perpendicular bisectory of ac is fac_mid + t * fac_dir
  // intersection of perpendicular bisectors:
  //    x=0.5  y= fac_mid.y + (0.5 - fac_mid.x) * fac_dir.y / fac_dir.x
  double fcenter_y;
  if (fac_dir.x() <= std::numeric_limits<double>::epsilon())
  {
    fcenter_y = fac_mid.y();
  }
  else
  {
    fcenter_y = fac_mid.y + (0.5 - fac_mid.x) * fac_dir.y / fac_dir.x;
  }

  // center of circle is at (0.5, fcenter_y) in 2D coord sys.  Put back into 3d coords
  center = (x * (0.5 + x0)) +
           (y * (fcenter_y + y0)) + 
           (n * z0);
  radius = (center - a).norm();
}


namespace
{
// calculate sphere bounding points using Emo Welzl's move-to-front algorithm.
struct SphereInfo
{
  std::vector<Eigen::Vector3d*> list_;
  std::vector<Eigen::Vector3d*> boundary_;
  Eigen::Vector3d center_;
  double radius_;

  SphereInfo(const EigenSTL::vector_Vector3d& points);
  genSphere(int nbound);
  findSphere(int npoints, int nbound);
};
}

void SphereInfo::SphereInfo(
      const EigenSTL::vector_Vector3d& points)
{
  int npoints = points.size();
  list_.resize(npoints);
  boundary_.resize(4);
  for (int i = 0 ; i < npoints ; i++)
    list_[i] = &points[i];
}

void SphereInfo::genCircle(
      Eigen::Vector3d& center,
      Eigen::Vector3d& normal,
      double& radius,
      bool colinear)
{


void SphereInfo::genSphere(
      int nbound)
{
  switch(nbound)
  {
  case 0:
    radius_ = 0;
    center_ = Eigen::Vector3d(0,0,0);
    break;
  case 1:
    radius_ = 0;
    center_ = boundary_[0];
    break;
  case 2:
    center_ = (boundary_[0] + boundary_[1]) * 0.5;
    radius_ = (boundary_[0] - center_).norm();
    break;
  case 3:
  case 4:
  default:
    logError("Bad nbound=%d for genSphere",nbound);
    radius_ = 0;
    center_ = Eigen::Vector3d(0,0,0);
    break;
  }
  
}

void SphereInfo::findSphere(
      int npoints,
      int nbound)
{

}


void robot_sphere_representation::generateBoundingSphere(
      const EigenSTL::vector_Vector3d& points,
      Eigen::Vector3d& center,
      double &radius)
{
  SphereInfo info(points);

  info.findSphere(points.size(), 0);

  center = info.center;
  radius = info.radius;
}
