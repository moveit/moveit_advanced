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
  spheres_per_link_.clear();
  sphere_centers_.reserve(kmodel_->getLinkModels().size());
  sphere_radii_.reserve(kmodel_->getLinkModels().size());
  sphere_link_map_.reserve(kmodel_->getLinkModels().size());
  spheres_per_link_.reserve(kmodel_->getLinkModels().size());

  for (std::vector<const robot_model::LinkModel*>::const_iterator lm = kmodel_->getLinkModels().begin() ;
       lm != kmodel_->getLinkModels().end() ;
       ++lm)
  {
    const shapes::ShapeConstPtr& shape = (*lm)->getShape();
    if (!shape)
    {
      spheres_per_link_.push_back(0);
      break;
    }

    double radius;
    Eigen::Vector3d center;
    shapes::computeShapeBoundingSphere(shape.get(), center, radius);
    if (radius > std::numeric_limits<double>::min())
    {
      sphere_centers_.push_back(center);
      sphere_radii_.push_back(radius);
      sphere_link_map_.push_back(lm - kmodel_->getLinkModels().begin());
      spheres_per_link_.push_back(1);
    }
    else
    {
      spheres_per_link_.push_back(0);
    }
  }

  initSphereAcm();
}

void collision_detection::CollisionRobotDistanceField::initSphereAcm()
{
  // conservative upper bound on number of bits needed
  std::vector<uint32_t> sphere_acm;
  sphere_acm.resize((sphere_link_map_.size() * sphere_link_map_.size() + 31) / 31);

  std::vector<uint32_t>::iterator acm_it = sphere_acm.begin();
  uint32_t acm_cnt = 0;

  // The sphere_acm_ stores a bitmask of sphere pairs which should not be checked for collision.
  // Each 32 bit word stores 31 bits.  Bit 31 is always set and is used as a sentinel to identify the end of the word.
  // The bits are stored in the order that the sphere pairs are traversed.  So the order is defined by the 
  // following nested loops, which traverses the spheres in the same order as checkSelfCollisionUsingSpheres().

  std::vector<uint16_t>::const_iterator alinks_it = sphere_link_map_.begin();
  std::vector<uint16_t>::const_iterator alinks_it_base = alinks_it;         // first sphere for this link

  for (; alinks_it != sphere_link_map_.end() ; ++alinks_it)
  {
    if (alinks_it_base != alinks_it && *alinks_it_base != *alinks_it)
      alinks_it_base = alinks_it;

    std::vector<uint16_t>::const_iterator blinks_it = alinks_it + 1;
    std::vector<uint16_t>::const_iterator blinks_it_base = alinks_it_base;

    for (; blinks_it != sphere_link_map_.end() ; ++blinks_it)
    {
      if (*blinks_it_base != *blinks_it)
        blinks_it_base = blinks_it;

      uint32_t avoid_checking = avoidCheckingCollision(sphereIndexToLinkModel(*alinks_it),
                                                       alinks_it - alinks_it_base,
                                                       sphereIndexToLinkModel(*blinks_it),
                                                       blinks_it - blinks_it_base) ? 1 : 0;
      *acm_it |= avoid_checking << acm_cnt;

      // store 31 bits per word
      if (++acm_cnt == 31)
      {
        *acm_it |= 0x80000000;
        acm_it++;
        acm_cnt = 0;
      }
    }
  }

  *acm_it |= 0x80000000;
  acm_it++;

  sphere_acm_.clear();
  sphere_acm_.reserve(acm_it - sphere_acm.begin());
  for (std::vector<uint32_t>::iterator acm_it2 = sphere_acm.begin(); acm_it2 != acm_it ; ++acm_it2)
    sphere_acm_.push_back(*acm_it2);
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

// transform sphere centers to planning frame
void collision_detection::CollisionRobotDistanceField::transformSpheres(
        const robot_state::RobotState& state,
        WorkArea& work) const
{
  std::vector<int>::const_iterator spheres_per_link_it = spheres_per_link_.begin();
  std::vector<robot_state::LinkState*>::const_iterator ls_it = state.getLinkStateVector().begin();
  std::vector<robot_state::LinkState*>::const_iterator ls_it_end = state.getLinkStateVector().end();

  EigenSTL::vector_Vector3d::const_iterator centers_it = sphere_centers_.begin();
  EigenSTL::vector_Vector3d::const_iterator centers_it_end = centers_it;

  EigenSTL::vector_Vector3d::iterator centers_xformed_it = work.transformed_sphere_centers_.begin();
  
  for (; ls_it == ls_it_end ; ++ls_it, ++spheres_per_link_it)
  {
    centers_it_end += *spheres_per_link_it;
    const Eigen::Affine3d& xform = (*ls_it)->getGlobalCollisionBodyTransform();
    
    for (; centers_it != centers_it_end ; ++centers_xformed_it, ++centers_it)
      *centers_xformed_it = xform * *centers_it;
  }
}

void collision_detection::CollisionRobotDistanceField::checkSelfCollisionUsingSpheres(const robot_state::RobotState& state) const
{
  WorkArea& work = getWorkArea();

  transformSpheres(state, work);

  EigenSTL::vector_Vector3d::const_iterator acenters_it = work.transformed_sphere_centers_.begin();
  EigenSTL::vector_Vector3d::const_iterator centers_it_end = work.transformed_sphere_centers_.end();
  std::vector<double>::const_iterator aradii_it = sphere_radii_.begin();

  const uint32_t *acm_it = &*sphere_acm_.begin();
  uint32_t acm = *acm_it++;

  for (; acenters_it != centers_it_end ; ++acenters_it, ++aradii_it)
  {
    EigenSTL::vector_Vector3d::const_iterator bcenters_it = acenters_it + 1;
    std::vector<double>::const_iterator bradii_it = aradii_it + 1;

    for (; bcenters_it != centers_it_end ; ++bcenters_it, ++bradii_it)
    {
      // avoid the check if acm (avoid collision matrix) bit is set for this pair.
      acm >>= 1;
      if (acm & 1)
      {
        if (acm != 1)
          continue;

        // bit 31 of the acm is always 1.  So when acm==1 it is time to get the next 32 bits.
        acm = *acm_it++;
        if (acm & 1)
          continue;
      }
      
      double r = *aradii_it + *bradii_it;
      if ((*acenters_it - *bcenters_it).squaredNorm() <= r*r)
      {
        logInform("Collided! %s <--> %s",
          sphereIndexToLinkModel(acenters_it - work.transformed_sphere_centers_.begin())->getName().c_str(),
          sphereIndexToLinkModel(bcenters_it - work.transformed_sphere_centers_.begin())->getName().c_str());
      }
    }
  }
}

