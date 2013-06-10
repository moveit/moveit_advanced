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

#ifndef MOVEIT_COLLISION_DETECTION_DISTANCE_FIELD_COLLISION_ROBOT_DISTANCE_FIEL
#define MOVEIT_COLLISION_DETECTION_DISTANCE_FIELD_COLLISION_ROBOT_DISTANCE_FIEL

#include <moveit/collision_detection_distance_field/collision_common_distance_field.h>
#include <boost/thread.hpp>

namespace collision_detection
{

class CollisionRobotDistanceField : public CollisionRobot
{
  friend class CollisionWorldDistanceField;

public:

  CollisionRobotDistanceField(const robot_model::RobotModelConstPtr &kmodel, double padding = 0.0, double scale = 1.0);
  CollisionRobotDistanceField(const CollisionRobotDistanceField &other);

  virtual void checkSelfCollision(const CollisionRequest &req,
                                  CollisionResult &res,
                                  const robot_state::RobotState &state) const;
  virtual void checkSelfCollision(const CollisionRequest &req,
                                  CollisionResult &res,
                                  const robot_state::RobotState &state,
                                  const AllowedCollisionMatrix &acm) const;
  virtual void checkSelfCollision(const CollisionRequest &req,
                                  CollisionResult &res,
                                  const robot_state::RobotState &state1,
                                  const robot_state::RobotState &state2) const;
  virtual void checkSelfCollision(const CollisionRequest &req,
                                  CollisionResult &res,
                                  const robot_state::RobotState &state1,
                                  const robot_state::RobotState &state2,
                                  const AllowedCollisionMatrix &acm) const;

  virtual void checkOtherCollision(const CollisionRequest &req,
                                   CollisionResult &res,
                                   const robot_state::RobotState &state,
                                   const CollisionRobot &other_robot,
                                   const robot_state::RobotState &other_state) const;
  virtual void checkOtherCollision(const CollisionRequest &req,
                                   CollisionResult &res,
                                   const robot_state::RobotState &state,
                                   const CollisionRobot &other_robot,
                                   const robot_state::RobotState &other_state,
                                   const AllowedCollisionMatrix &acm) const;
  virtual void checkOtherCollision(const CollisionRequest &req,
                                   CollisionResult &res,
                                   const robot_state::RobotState &state1,
                                   const robot_state::RobotState &state2,
                                   const CollisionRobot &other_robot,
                                   const robot_state::RobotState &other_state1,
                                   const robot_state::RobotState &other_state2) const;
  virtual void checkOtherCollision(const CollisionRequest &req,
                                   CollisionResult &res,
                                   const robot_state::RobotState &state1,
                                   const robot_state::RobotState &state2,
                                   const CollisionRobot &other_robot,
                                   const robot_state::RobotState &other_state1,
                                   const robot_state::RobotState &other_state2,
                                   const AllowedCollisionMatrix &acm) const;

  virtual double distanceSelf(const robot_state::RobotState &state) const;
  virtual double distanceSelf(const robot_state::RobotState &state,
                              const AllowedCollisionMatrix &acm) const;
  virtual double distanceOther(const robot_state::RobotState &state,
                               const CollisionRobot &other_robot,
                               const robot_state::RobotState &other_state) const;
  virtual double distanceOther(const robot_state::RobotState &state,
                               const CollisionRobot &other_robot,
                               const robot_state::RobotState &other_state,
                               const AllowedCollisionMatrix &acm) const;

public: /* DEBUGGING functions */

  // return the collision spheres for a link in link's collision geometry frame.
  void getLinkSpheres(const std::string& link_name,
                      EigenSTL::vector_Vector3d& centers,
                      std::vector<double>& radii) const;

protected:

  virtual void updatedPaddingOrScaling(const std::vector<std::string> &links);


  

private:

  // These types are 16 bits to minimize the size of various arrays for
  // performance reasons.  If more than 65536 spheres or links are needed the
  // size can be increased here.
  typedef uint16_t SphereIndex;
  typedef uint16_t LinkIndex;

  //###########################################################################
  //############################### WORK AREA #################################
  //###########################################################################

  /// Contains temporary data used during a collision query.
  // Instead of allocating this every query, a single instance is allocated per
  // thread and used for all queries from that thread.
  struct WorkArea
  {
    ~WorkArea();

    // initialize query
    void initQuery(const char* descrip,
                   const CollisionRequest *req,
                   CollisionResult *res,
                   const robot_state::RobotState *state1,
                   const robot_state::RobotState *state2,
                   const CollisionRobot *other_robot,
                   const robot_state::RobotState *other_state1,
                   const robot_state::RobotState *other_state2,
                   const AllowedCollisionMatrix *acm);

    // place to store transformed copy of sphere_centers_
    EigenSTL::vector_Vector3d transformed_sphere_centers_;

    // copies of collision query parameters
    const CollisionRequest *req_;
    CollisionResult *res_;
    const robot_state::RobotState *state1_;
    const robot_state::RobotState *state2_;
    const CollisionRobot *other_robot_;
    const robot_state::RobotState *other_state1_;
    const robot_state::RobotState *other_state2_;
    const AllowedCollisionMatrix *acm_;

    // true if any of the collisions we saw have a DecideContactFn
    bool found_conditional_contact_;
  };


  // get a mutable per-thread work area to store temporary values
  WorkArea& getWorkArea() const;

  //###########################################################################
  //############################### ACCESSOR CONVENIENCE FUNCTIONS ############
  //###########################################################################

  // return the LinkModel for a link index
  const robot_model::LinkModel* linkIndexToLinkModel(int link_index) const
  {
    assert(link_index >= 0 && link_index < link_order_.size());
    return kmodel_->getLinkModels()[link_order_[link_index]];
  }

  // return the LinkState for a link index
  robot_state::LinkState* linkIndexToLinkState(int link_index, const robot_state::RobotState* state) const
  {
    assert(link_index >= 0 && link_index < link_order_.size());
    return state->getLinkStateVector()[link_order_[link_index]];
  }

  // return name for a link index
  const std::string& linkIndexToName(int link_index) const
  {
    return linkIndexToLinkModel(link_index)->getName();
  }




  // return the link_index of link owning sphere with sphere_index
  int sphereIndexToLinkIndex(int sphere_index) const
  {
    assert(sphere_index >= 0 && sphere_index < sphere_idx_to_link_index_.size());
    return sphere_idx_to_link_index_[sphere_index];
  }

  // return the LinkModel for a particular sphere index in sphere_centers_, sphere_radii_, or sphere_link_map_
  const robot_model::LinkModel* sphereIndexToLinkModel(int sphere_index) const
  {
    return linkIndexToLinkModel(sphereIndexToLinkIndex(sphere_index));
  }

  // lookup index for a link. 
  //   -1 if link has no geometry or does not exist.
  const int linkNameToIndex(const std::string& link_name) const;

#if 0
  // number of links in robot
  const std::size_t linkCount() const
  {
    return kmodel_->getLinkModels().size();
  }
#endif

  // find a link's collision spheres in the srdf
  const srdf::Model::LinkSpheres *getSrdfLinkSpheres(const std::string& link) const;


  //###########################################################################
  //############################### INITIALIZATION ############################
  //###########################################################################

  // initialize everything.  Called from constructors.
  void initialize();

  void initParams();

  //###########################################################################
  //############################### SPHERE COLLISION ##########################
  //###########################################################################

  // sphere init functions
  void initSpheres();
  void initSphereAcm();
  bool avoidCheckingCollision(
        const robot_model::LinkModel* link0,
        size_t sphere_idx0,
        const robot_model::LinkModel* link1,
        size_t sphere_idx1);


  // transform sphere centers to planning frame.  Results are placed into mutable work.transformed_sphere_centers_.
  void transformSpheres(
        const robot_state::RobotState& state,
        WorkArea& work) const;

  // check self collision using only sphere checks
  void checkSelfCollisionUsingSpheres(WorkArea& work) const;

  // helpers for checkSelfCollisionUsingSpheres()
  template<class Collision>
    bool checkSelfCollisionUsingSpheresLoop(WorkArea& work, const SphereIndex *sphere_list) const;
  bool checkSpherePairAll(WorkArea& work,
                          int a_idx,
                          int b_idx,
                          const Eigen::Vector3d& a_center,
                          const Eigen::Vector3d& b_center,
                          double a_radius,
                          double b_radius,
                          double dsq) const;
  class CollisionBool;
  class CollisionAll;
  friend class CollisionBool;
  friend class CollisionAll;

  // helpers for when a sphere collision is detected
  AllowedCollision::Type getLinkPairAcmType(WorkArea& work,
                                            int a_link,
                                            int b_link) const;
  void addSphereContact(WorkArea& work,
                        int a_link,
                        int b_link,
                        const Eigen::Vector3d& a_center,
                        const Eigen::Vector3d& b_center,
                        double a_radius,
                        double b_radius,
                        double dsq) const;
  void addSphereCost(WorkArea& work,
                     int a_idx,
                     int b_idx) const;

  //###########################################################################
  //############################### INTRA GROUP DF COLLISION ##################
  //###########################################################################



  //###########################################################################
  //############################### DATA ######################################
  //###########################################################################

  //===========================================================================
  // Configuration
  //===========================================================================

  // This affects size and generation of distance fields.  Changing it requires
  // recalculating all distance fields from scratch.
  //
  // This sets an upper bound on the size of sphere that can be checked with
  // distance field calculation.
  //
  // This also sets an upper bound on results returned by distance*() queries.
  double max_df_distance_;  

  //===========================================================================
  // Sphere data
  //===========================================================================
  
  // This defines the "link order".  It contains only links with collision geometry.
  // The position in the array is the LinkIndex.
  // The value stored is the index into the RobotModel::getLinkModels() and
  // RobotState::getLinkStateVector() arrays.
  std::vector<int> link_order_;

  // This map allows looking up a links_order_index from a link name.
  // Links with no collision geometry are included in this list and contain an index of -1.
  std::map<std::string,int> link_name_to_link_index_;

  // Table of link indices and spheres. Used to transform spheres.  See initSpheres() for details.
  std::vector<SphereIndex> sphere_transform_indices_;

  // sphere centers for all spheres for all links in link_order_
  EigenSTL::vector_Vector3d sphere_centers_;

  // sphere radii for all spheres for all links in link_order_
  std::vector<double> sphere_radii_;

  // for each sphere, index in link_order_ of link the sphere is associated with.
  std::vector<LinkIndex> sphere_idx_to_link_index_;

  // Table of spheres to check for collisions.  See initSphereAcm() for details.
  std::vector<SphereIndex> self_collide_list_;

  // bounding sphere center for each link in link_order_
  EigenSTL::vector_Vector3d bounding_sphere_centers_;

  // bounding sphere radius for each link in link_order_
  std::vector<double> bounding_sphere_radii_;

  // radius of largest bounding sphere
  double max_bounding_sphere_radius_;

  //===========================================================================
  // Work area
  //===========================================================================
  
  // mutable thread specific work area
  mutable boost::thread_specific_ptr<WorkArea> work_area_;
};

}

#endif
