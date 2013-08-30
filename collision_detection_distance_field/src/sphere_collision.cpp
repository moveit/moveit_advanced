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
 *   * Neither the name of Willow Garage nor the names of its
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
#include "collision_robot_distance_field_inline.h"
#include <moveit/robot_sphere_representation/body_bounding_sphere.h>
#include <console_bridge/console.h>
#include <cassert>


const srdf::Model::LinkSpheres *collision_detection::CollisionRobotDistanceField::getSrdfLinkSpheres(const std::string& link_name) const
{
  for (std::vector<srdf::Model::LinkSpheres>::const_iterator lsp = robot_model_->getSRDF()->getLinkSphereApproximations().begin() ;
       lsp != robot_model_->getSRDF()->getLinkSphereApproximations().end() ;
       ++lsp)
    if (lsp->link_ == link_name)
      return &*lsp;

  return NULL;
}


void collision_detection::CollisionRobotDistanceField::getLinkSpheres(
      const std::string& link_name,
      EigenSTL::vector_Vector3d& centers,
      std::vector<double>& radii) const
{
  int link_idx = linkNameToIndex(link_name);

  centers.clear();
  radii.clear();
  for (std::size_t i = 0 ; i < sphere_idx_to_link_index_.size() ; ++i)
  {
    if (sphere_idx_to_link_index_[i] == link_idx)
    {
      centers.push_back(sphere_centers_[i]);
      radii.push_back(sphere_radii_[i]);
    }
  }
}

void collision_detection::CollisionRobotDistanceField::getLinkBoundingSphere(
      const std::string& link_name,
      Eigen::Vector3d& center,
      double& radius) const
{
  int link_idx = linkNameToIndex(link_name);
  if (link_idx >= 0)
  {
    center = bounding_sphere_centers_[link_idx];
    radius = bounding_sphere_radii_[link_idx];
  }
  else
  {
    center = Eigen::Vector3d(0,0,0);
    radius = 0;
  }
}

void collision_detection::CollisionRobotDistanceField::initSpheres()
{
  int sphere_cnt = 0;
  int link_cnt = robot_model_->getLinkModels().size();

  // count how many spheres we will need
  for (std::vector<const robot_model::LinkModel*>::const_iterator lm = robot_model_->getLinkModels().begin() ;
       lm != robot_model_->getLinkModels().end() ;
       ++lm)
  {
    if (!(*lm)->getShapes().empty())
      continue;

    const srdf::Model::LinkSpheres *lsp = getSrdfLinkSpheres((*lm)->getName());

    if (lsp->spheres_.empty())
    {
      sphere_cnt++;
    }
    else if (lsp->spheres_.size() == 1 && lsp->spheres_.begin()->radius_ <= std::numeric_limits<double>::min())
    {
       // only radius=0 sphere
    }
    else
    {
      sphere_cnt += lsp->spheres_.size();
    }
  }

  sphere_centers_.clear();
  sphere_radii_.clear();
  sphere_idx_to_link_index_.clear();
  sphere_transform_indices_.clear();
  link_order_.clear();
  link_name_to_link_index_.clear();
  bounding_sphere_centers_.clear();
  bounding_sphere_radii_.clear();

  sphere_centers_.reserve(sphere_cnt);
  sphere_radii_.reserve(sphere_cnt);
  sphere_idx_to_link_index_.reserve(sphere_cnt);
  sphere_transform_indices_.reserve(link_cnt + sphere_cnt + 1);
  link_order_.reserve(link_cnt);
  bounding_sphere_centers_.reserve(link_cnt);
  bounding_sphere_radii_.reserve(link_cnt);

  max_bounding_sphere_radius_ = 0.0;

  // sphere_transform_indices_ lists all spheres and how they are transformed.  Format is:
  //    cnt=N         -- number of spheres for this link
  //    link_index    -- index of link in RobotState::getLinkStateVector()
  //    ...
  //
  // list is terminated with cnt==0

  int link_model_index = 0;
  for (std::vector<const robot_model::LinkModel*>::const_iterator lm = robot_model_->getLinkModels().begin() ;
       lm != robot_model_->getLinkModels().end() ;
       ++lm, ++link_model_index)
  {
    // initialize link to -1.  This will be the value if the link has no collision geometry.
    link_name_to_link_index_[(*lm)->getName()] = -1;

    LinkIndex link_index = link_order_.size();

    if ((*lm)->getShapes().empty())
      continue;
    // \todo make this work with multiple shapes 

    // Calculate bounding sphere for entire link
    const shapes::ShapeConstPtr& shape = (*lm)->getShapes()[0];
    
    double bounding_radius;
    Eigen::Vector3d bounding_center;
    robot_sphere_representation::findTightBoundingSphere(*shape, bounding_center, bounding_radius);
    if (bounding_radius <= std::numeric_limits<double>::epsilon())
      continue;

    double scale = 1.0;
    std::map<std::string, double>::const_iterator scale_it = link_scale_.find((*lm)->getName());
    if (scale_it != link_scale_.end())
      scale = scale_it->second;

    if (scale <= std::numeric_limits<double>::epsilon())
    {
      logError("Scale=%f <= 0 for link %s.", scale, (*lm)->getName().c_str());
      continue;
    }
    if (scale < 1.0)
    {
      logWarn("Scale=%f < 1.0 for link %s.", scale, (*lm)->getName().c_str());
    }

    // If srdf_model has any spheres:
    //   Use them.  Eliminate all spheres with radius<=0.  If none are left then
    //   this link has no collidable geometry and will not be considered for
    //   sphere-based collision detection.
    const srdf::Model::LinkSpheres *lsp = getSrdfLinkSpheres((*lm)->getName());

    int sphere_cnt = 0;
    if (lsp && !lsp->spheres_.empty())
    {
      for (std::vector<srdf::Model::Sphere>::const_iterator sp = lsp->spheres_.begin() ;
           sp != lsp->spheres_.end() ;
           ++sp)
      {
        if (sp->radius_ > std::numeric_limits<double>::min())
        {
          Eigen::Vector3d center(sp->center_x_,
                                 sp->center_y_,
                                 sp->center_z_);
          sphere_centers_.push_back(center * scale);
          sphere_radii_.push_back(sp->radius_ * scale);
          sphere_idx_to_link_index_.push_back(link_index);
          sphere_cnt++;
        }
      }
    }
    else
    {
      // If srdf_model does not have ANY spheres:
      //   Use bounding sphere as the sole collision sphere.
      sphere_centers_.push_back(bounding_center);
      sphere_radii_.push_back(bounding_radius);
      sphere_idx_to_link_index_.push_back(link_index);
      sphere_cnt = 1;
    }

    // only add the link if there is geometry (i.e. sphere_cnt>=1)
    if (sphere_cnt)
    {
      sphere_transform_indices_.push_back(sphere_cnt);
      sphere_transform_indices_.push_back(link_model_index);

      link_name_to_link_index_[(*lm)->getName()] = link_index;
      link_order_.push_back(link_model_index);

      bounding_sphere_centers_.push_back(bounding_center * scale);
      bounding_sphere_radii_.push_back(bounding_radius * scale);

      max_bounding_sphere_radius_ = std::max(max_bounding_sphere_radius_, bounding_radius * scale);
    }
  }

  // terminate list with cnt=0
  sphere_transform_indices_.push_back(0);

  initSphereAcm();
}

void collision_detection::CollisionRobotDistanceField::initSphereAcm()
{
  std::vector<SphereIndex> self_collide_list;
  self_collide_list.resize(sphere_idx_to_link_index_.size() * (3 + sphere_idx_to_link_index_.size()) + 1);
  std::vector<SphereIndex>::iterator acm_it = self_collide_list.begin();

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

  std::vector<LinkIndex>::const_iterator alinks_it = sphere_idx_to_link_index_.begin();
  std::vector<LinkIndex>::const_iterator alinks_it_base = alinks_it;         // first sphere for this link

  for (; alinks_it != sphere_idx_to_link_index_.end() ; ++alinks_it)
  {
    if (alinks_it_base != alinks_it && *alinks_it_base != *alinks_it)
      alinks_it_base = alinks_it;

    std::vector<LinkIndex>::const_iterator blinks_it = alinks_it + 1;
    std::vector<LinkIndex>::const_iterator blinks_it_base = alinks_it_base;

    std::vector<SphereIndex>::iterator acm_it_cnt = acm_it++;
    *acm_it++ = alinks_it - sphere_idx_to_link_index_.begin();

    for (; blinks_it != sphere_idx_to_link_index_.end() ; ++blinks_it)
    {
      if (*blinks_it_base != *blinks_it)
        blinks_it_base = blinks_it;

      if (avoidCheckingCollision(linkIndexToLinkModel(*alinks_it),
                                 alinks_it - alinks_it_base,
                                 linkIndexToLinkModel(*blinks_it),
                                 blinks_it - blinks_it_base))
        continue;

      *acm_it++ = blinks_it - sphere_idx_to_link_index_.begin();
    }

    int cnt = acm_it - acm_it_cnt - 2;
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
  for (std::vector<SphereIndex>::iterator acm_it2 = self_collide_list.begin(); acm_it2 != acm_it ; ++acm_it2)
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
  for (std::vector<srdf::Model::DisabledCollision>::const_iterator it = robot_model_->getSRDF()->getDisabledCollisionPairs().begin() ;
       it != robot_model_->getSRDF()->getDisabledCollisionPairs().end() ;
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
  EigenSTL::vector_Vector3d::const_iterator centers_it = sphere_centers_.begin();
  EigenSTL::vector_Vector3d::iterator centers_xformed_it = work.transformed_sphere_centers_.begin();

  const SphereIndex *idx_it = &*sphere_transform_indices_.begin();

  for (int cnt = *idx_it++ ; cnt ; cnt = *idx_it++)
  {
    int link_idx = *idx_it++;
    const Eigen::Affine3d& xform = state.getCollisionBodyTransform(robot_model_->getLinkModels()[link_idx], 0);

    do
    {
      *centers_xformed_it++ = xform * *centers_it++;
    }
    while (--cnt);
  }
}

template<class Collision>
bool collision_detection::CollisionRobotDistanceField::checkSelfCollisionUsingSpheresLoop(
    WorkArea& work,
    const SphereIndex *sphere_list) const
{
  // walk the list of collidable spheres and check for collisions
  const SphereIndex *acm_it = sphere_list;
  for (int cnt = *acm_it++ ; cnt ; cnt = *acm_it++)
  {
    int a_idx = *acm_it++;
    const Eigen::Vector3d& a_center = work.transformed_sphere_centers_[a_idx];
    double a_radius = sphere_radii_[a_idx];

    do
    {
      int b_idx = *acm_it++;
      const Eigen::Vector3d& b_center = work.transformed_sphere_centers_[b_idx];
      double b_radius = sphere_radii_[b_idx];

      double r = a_radius + b_radius;
      double dsq = (a_center - b_center).squaredNorm();
      if (dsq <= r*r)
      {
        logInform("     COLLIDED! %s <--> %s",
          sphereIndexToLinkModel(a_idx)->getName().c_str(),
          sphereIndexToLinkModel(b_idx)->getName().c_str());
        if (Collision::check(this, work, a_idx, b_idx, a_center, b_center, a_radius, b_radius, dsq))
          return true;
      }
    }
    while (--cnt);
  }

  return false;
}

// add sphere pair contact to work.req_->contacts
// a_idx < b_idx.
void collision_detection::CollisionRobotDistanceField::addSphereContact(
    WorkArea& work,
    int a_link,
    int b_link,
    const Eigen::Vector3d& a_center,
    const Eigen::Vector3d& b_center,
    double a_radius,
    double b_radius,
    double dsq) const
{
  const std::string& a_name = linkIndexToName(a_link);
  const std::string& b_name = linkIndexToName(b_link);

  std::pair<std::string, std::string> names(a_name, b_name);
  std::vector<Contact>& contacts = work.res_->contacts[names];
  int idx = contacts.size();

  if (idx >= work.req_->max_contacts_per_pair)
    return;

  contacts.resize(idx+1);
  Contact& contact = contacts[idx];

  contact.body_name_1 = a_name;
  contact.body_name_2 = b_name;
  contact.body_type_1 = collision_detection::BodyTypes::ROBOT_LINK;
  contact.body_type_2 = collision_detection::BodyTypes::ROBOT_LINK;

  if (dsq > std::numeric_limits<double>::epsilon())
  {
    double dist = std::sqrt(dsq);
    double oodist = 1.0/dist;
    double ratio = a_radius / (a_radius + b_radius);
    contact.normal = (b_center - a_center) * oodist;
    contact.pos = a_center + ratio * dist * contact.normal;
    contact.depth = a_radius + b_radius - dist;
  }
  else
  {
    contact.normal.setZero();
    contact.pos = a_center;
    contact.depth = a_radius + b_radius;
  }

  work.res_->contact_count++;

  if (work.df_contacts_)
  {
    work.df_contacts_->resize(work.df_contacts_->size()+1);
    DFContact *dfcontact = &work.df_contacts_->back();
    dfcontact->copyFrom(contact);  // This clears all fields.
    dfcontact->sphere_center_1 = a_center;
    dfcontact->sphere_radius_1 = a_radius;
    dfcontact->sphere_center_2 = b_center;
    dfcontact->sphere_radius_2 = b_radius;
  }
}

// add cost of sphere-pair to work.req_->cost_sources
// a_idx < b_idx.
void collision_detection::CollisionRobotDistanceField::addSphereCost(
    WorkArea& work,
    int a_idx,
    int b_idx) const
{
  // TODO
}

// Indicates whether the collision between 2 links should be ignored.
// See collision_matrix.h for details.
// a_idx < b_idx.
collision_detection::AllowedCollision::Type collision_detection::CollisionRobotDistanceField::getLinkPairAcmType(
    WorkArea& work,
    int a_link,
    int b_link) const
{
  AllowedCollision::Type allowed_collision;
  if (!work.acm_->getAllowedCollision(linkIndexToName(a_link), linkIndexToName(b_link), allowed_collision))
    allowed_collision = collision_detection::AllowedCollision::NEVER;
  return allowed_collision;
}

// Bool is simplest query.  Return true if any collision occurs.
class collision_detection::CollisionRobotDistanceField::CollisionBool
{
public:
  static bool check(
      const collision_detection::CollisionRobotDistanceField* crobot,
      collision_detection::CollisionRobotDistanceField::WorkArea& work,
      int a_idx,
      int b_idx,
      const Eigen::Vector3d& a_center,
      const Eigen::Vector3d& b_center,
      double a_radius,
      double b_radius,
      double dsq)
  {
    return true;
  }
};

// General query.  Call checkSpherePairAll() to handle general case.
class collision_detection::CollisionRobotDistanceField::CollisionAll
{
public:
  static bool check(
      const collision_detection::CollisionRobotDistanceField* crobot,
      collision_detection::CollisionRobotDistanceField::WorkArea& work,
      int a_idx,
      int b_idx,
      const Eigen::Vector3d& a_center,
      const Eigen::Vector3d& b_center,
      double a_radius,
      double b_radius,
      double dsq)
  {
    return crobot->checkSpherePairAll(work, a_idx, b_idx, a_center, b_center, a_radius, b_radius, dsq);
  }
};

bool collision_detection::CollisionRobotDistanceField::checkSpherePairAll(
    WorkArea& work,
    int a_idx,
    int b_idx,
    const Eigen::Vector3d& a_center,
    const Eigen::Vector3d& b_center,
    double a_radius,
    double b_radius,
    double dsq) const
{
  int a_link = sphere_idx_to_link_index_[a_idx];
  int b_link = sphere_idx_to_link_index_[b_idx];

  int max_contacts = work.req_->contacts ? work.req_->max_contacts : 0;

  if (work.acm_)
  {
    collision_detection::AllowedCollision::Type type = getLinkPairAcmType(work, a_link, b_link);
    if (type == collision_detection::AllowedCollision::ALWAYS)
    {
      // TODO: skip acm_it to next link
      return false;
    }
    if (type == collision_detection::AllowedCollision::CONDITIONAL)
    {
      max_contacts = std::numeric_limits<int>::max();
      work.found_conditional_contact_ = true;
    }
  }

  work.res_->collision = true;

  if (work.res_->contact_count < max_contacts)
  {
    addSphereContact(work, a_link, b_link, a_center, b_center, a_radius, b_radius, dsq);
  }

  if (work.req_->cost)
    addSphereCost(work, a_idx, b_idx);

  return false;
}

void collision_detection::CollisionRobotDistanceField::checkSelfCollisionUsingSpheres(
    WorkArea& work) const
{
  transformSpheres(*work.state1_, work);

  // decide which spheres to check
  const SphereIndex *sphere_list = &*self_collide_list_.begin();
  if (!work.req_->group_name.empty())
  {
    // TODO: use individual self_collide_list based on req.group_name
  }

  if (!work.acm_ && !work.req_->distance && !work.req_->cost && !work.req_->contacts)
  {
    work.res_->collision = checkSelfCollisionUsingSpheresLoop<CollisionBool>(work, sphere_list);
    return;
  }

  work.found_conditional_contact_ = false;

  // common case does all the bells and whistles
  work.res_->collision = checkSelfCollisionUsingSpheresLoop<CollisionAll>(work, sphere_list);

  if (work.found_conditional_contact_)
  {
    logError("Conditional contacts not supported yet");
  }
}
