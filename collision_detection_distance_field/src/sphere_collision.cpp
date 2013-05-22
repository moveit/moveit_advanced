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

#include <moveit/collision_detection_distance_field/collision_robot_distance_field.h>
#include <geometric_shapes/shape_operations.h>
#include <console_bridge/console.h>
#include <cassert>

collision_detection::CollisionRobotDistanceField::WorkArea& collision_detection::CollisionRobotDistanceField::getWorkArea() const
{
  if (!work_area_.get())
  {
    work_area_.reset(new WorkArea);
    WorkArea& work = *work_area_;
    work.transformed_sphere_centers_.resize(sphere_centers_.size());
  }
  return *work_area_;
}

void collision_detection::CollisionRobotDistanceField::initSpheres()
{
  // for now just use bounding sphere for each link

  sphere_centers_.clear();
  sphere_radii_.clear();
  sphere_link_map_.clear();
  sphere_transform_indices_.clear();
  sphere_centers_.reserve(kmodel_->getLinkModels().size());
  sphere_radii_.reserve(kmodel_->getLinkModels().size());
  sphere_link_map_.reserve(kmodel_->getLinkModels().size());
  sphere_transform_indices_.reserve(kmodel_->getLinkModels().size());

  // sphere_transform_indices_ lists all spheres and how they are transformed.  Format is:
  //    cnt=N         -- number of spheres for this link
  //    link_index    -- index of link
  //    ...
  //
  // list is terminated with cnt==0

  for (std::vector<const robot_model::LinkModel*>::const_iterator lm = kmodel_->getLinkModels().begin() ;
       lm != kmodel_->getLinkModels().end() ;
       ++lm)
  {
    const shapes::ShapeConstPtr& shape = (*lm)->getShape();
    if (!shape)
      continue;

    double radius;
    Eigen::Vector3d center;
    shapes::computeShapeBoundingSphere(shape.get(), center, radius);
    if (radius > std::numeric_limits<double>::min())
    {
      sphere_centers_.push_back(center);
      sphere_radii_.push_back(radius);
      sphere_link_map_.push_back(lm - kmodel_->getLinkModels().begin());
      sphere_transform_indices_.push_back(1);
      sphere_transform_indices_.push_back(lm - kmodel_->getLinkModels().begin());
    }
  }

  // terminate list with cnt=0
  sphere_transform_indices_.push_back(0);

  initSphereAcm();
}

void collision_detection::CollisionRobotDistanceField::initSphereAcm()
{
  std::vector<uint16_t> self_collide_list;
  self_collide_list.resize(sphere_link_map_.size() * (3 + sphere_link_map_.size()) + 1);
  std::vector<uint16_t>::iterator acm_it = self_collide_list.begin();

  // The self_collide_list_ stores pairs of spheres which should be checked for collision in a
  // format that is efficient for traversal.
  //
  // Each sphere that should be checked has an entry that looks like this:
  //    cnt = N
  //    sphere_index
  //      other_sphere_index_1
  //      other_sphere_index_2
  //      ...              
  //      other_sphere_index_N
  //
  // The last entry (and only the last entry) has a cnt==0

  std::vector<uint16_t>::const_iterator alinks_it = sphere_link_map_.begin();
  std::vector<uint16_t>::const_iterator alinks_it_base = alinks_it;         // first sphere for this link

  for (; alinks_it != sphere_link_map_.end() ; ++alinks_it)
  {
    if (alinks_it_base != alinks_it && *alinks_it_base != *alinks_it)
      alinks_it_base = alinks_it;

    std::vector<uint16_t>::const_iterator blinks_it = alinks_it + 1;
    std::vector<uint16_t>::const_iterator blinks_it_base = alinks_it_base;

    std::vector<uint16_t>::iterator acm_it_cnt = acm_it++;
    *acm_it++ = alinks_it - sphere_link_map_.begin();

logInform("  idx=%d", acm_it_cnt - self_collide_list.begin());
logInform("  sphere %d", alinks_it - sphere_link_map_.begin());

    for (; blinks_it != sphere_link_map_.end() ; ++blinks_it)
    {
      if (*blinks_it_base != *blinks_it)
        blinks_it_base = blinks_it;

      if (avoidCheckingCollision(kmodel_->getLinkModels()[*alinks_it],
                                 alinks_it - alinks_it_base,
                                 kmodel_->getLinkModels()[*blinks_it],
                                 blinks_it - blinks_it_base))
        continue;

      *acm_it++ = blinks_it - sphere_link_map_.begin();
logInform("        check sphere %d", blinks_it - sphere_link_map_.begin());
    }

    int cnt = acm_it - acm_it_cnt - 2;
logInform("        cnt          %d", cnt);
    if (cnt > 0)
      *acm_it_cnt = cnt;
    else
      acm_it = acm_it_cnt;
  }

  // add last entry with cnt==0
  *acm_it++ = 0;

  assert(acm_it - self_collide_list.begin() < self_collide_list.size());

  self_collide_list_.clear();
  self_collide_list_.reserve(acm_it - self_collide_list.begin());
  for (std::vector<uint16_t>::iterator acm_it2 = self_collide_list.begin(); acm_it2 != acm_it ; ++acm_it2)
    self_collide_list_.push_back(*acm_it2);
}

// Check whether collision checking should be avoided for this sphere pair.
// return true if these spheres should never be checked for collisions.
// return false if these spheres should be checked for collisions.
bool collision_detection::CollisionRobotDistanceField::avoidCheckingCollision(
        const robot_model::LinkModel* link0,
        size_t sphere_idx0,
        const robot_model::LinkModel* link1,
        size_t sphere_idx1)
{
  // avoid if spheres are part of same link
  if (link0->getName() == link1->getName())
    return true;

  // check default collision matrix from srdf
  for (std::vector<srdf::Model::DisabledCollision>::const_iterator it = kmodel_->getSRDF()->getDisabledCollisionPairs().begin() ;
       it != kmodel_->getSRDF()->getDisabledCollisionPairs().end() ;
       ++it)
  {
    if (link0->getName() == it->link1_)
    {
      if (link1->getName() == it->link2_)
        return true;
    }
    else if (link0->getName() == it->link2_)
    {
      if (link1->getName() == it->link1_)
        return true;
    }
  }

  return false;
}

static std::string poseString(const Eigen::Affine3d& pose, const std::string& pfx = "")
{
  std::stringstream ss;
  ss.precision(3);
  for (int y=0;y<4;y++)
  {
    ss << pfx;
    for (int x=0;x<4;x++)
    {
      ss << std::setw(8) << pose(y,x) << " ";
    }
    ss << std::endl;
  }
  return ss.str();
}



// transform sphere centers to planning frame
void collision_detection::CollisionRobotDistanceField::transformSpheres(
        const robot_state::RobotState& state,
        WorkArea& work) const
{
  EigenSTL::vector_Vector3d::const_iterator centers_it = sphere_centers_.begin();
  EigenSTL::vector_Vector3d::iterator centers_xformed_it = work.transformed_sphere_centers_.begin();

  const uint16_t *idx_it = &*sphere_transform_indices_.begin();

  for (int cnt = *idx_it++ ; cnt ; cnt = *idx_it++)
  {
    int link_idx = *idx_it++;
    const robot_state::LinkState* link = state.getLinkStateVector()[link_idx];
    const Eigen::Affine3d& xform = link->getGlobalCollisionBodyTransform();

    logInform("\n%s",poseString(xform, link->getName().c_str()).c_str());

    do
    {
      *centers_xformed_it++ = xform * *centers_it++;
logInform("BEGINX checkSelfCollisionUsingSpheres -- call transformSpheres()");
    }
    while (--cnt);
logInform("BEGINY checkSelfCollisionUsingSpheres -- call transformSpheres()");
  }
logInform("BEGINZ checkSelfCollisionUsingSpheres -- call transformSpheres()");
}

void collision_detection::CollisionRobotDistanceField::checkSelfCollisionUsingSpheres(const robot_state::RobotState& state) const
{
  WorkArea& work = getWorkArea();

logInform("BEGIN1 checkSelfCollisionUsingSpheres -- call transformSpheres()");
logInform("BEGIN2 checkSelfCollisionUsingSpheres -- call transformSpheres()");
logInform("BEGIN3 checkSelfCollisionUsingSpheres -- call transformSpheres()");
logInform("BEGIN4 checkSelfCollisionUsingSpheres -- call transformSpheres()");
logInform("BEGIN5 checkSelfCollisionUsingSpheres -- call transformSpheres()");
logInform("BEGIN6 checkSelfCollisionUsingSpheres -- call transformSpheres()");
logInform("BEGIN7 checkSelfCollisionUsingSpheres -- call transformSpheres()");
logInform("BEGIN8 checkSelfCollisionUsingSpheres -- call transformSpheres()");
logInform("BEGIN9 checkSelfCollisionUsingSpheres -- call transformSpheres()");
  transformSpheres(state, work);
logInform("BEGIN checkSelfCollisionUsingSpheres -- post transformSpheres()");

  // walk the list of collidable spheres and check for collisions
  const uint16_t *acm_it = &*self_collide_list_.begin();
  for (int cnt = *acm_it++ ; cnt ; cnt = *acm_it++)
  {
logInform("--cnt=%d",cnt);
    int a_idx = *acm_it++;
    const Eigen::Vector3d& a_center = work.transformed_sphere_centers_[a_idx];
    double a_radius = sphere_radii_[a_idx];

logInform("        SPHERE %3d   (%f %f %f) %f", a_idx, a_center.x(), a_center.y(), a_center.z(), a_radius);
    do
    {
      int b_idx = *acm_it++;
      const Eigen::Vector3d& b_center = work.transformed_sphere_centers_[b_idx];
      double b_radius = sphere_radii_[b_idx];
logInform("        sphere %3d   (%f %f %f) %f", b_idx, b_center.x(), b_center.y(), b_center.z(), b_radius);

      double r = a_radius + b_radius;
logInform("        check %36s %36s     r=%f  r*r=%f  cc=%f",
  sphereIndexToLinkModel(a_idx)->getName().c_str(),
  sphereIndexToLinkModel(b_idx)->getName().c_str(),
  r, r*r, (a_center - b_center).squaredNorm());
      if ((a_center - b_center).squaredNorm() <= r*r)
      {
        logInform("     COLLIDED! %s <--> %s",
          sphereIndexToLinkModel(a_idx)->getName().c_str(),
          sphereIndexToLinkModel(b_idx)->getName().c_str());
      }
    }
    while (--cnt);
  }
}

