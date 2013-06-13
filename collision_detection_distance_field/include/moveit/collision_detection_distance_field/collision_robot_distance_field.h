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

#ifndef MOVEIT_COLLISION_DETECTION_DISTANCE__FIELD_COLLISION_ROBOT_DISTANCE_FIELD
#define MOVEIT_COLLISION_DETECTION_DISTANCE__FIELD_COLLISION_ROBOT_DISTANCE_FIELD

#include <moveit/collision_detection_distance_field/collision_common_distance_field.h>
#include <moveit/collision_detection_distance_field/static_distance_field.h>
#include <moveit/collision_detection_distance_field/bit_array.h>
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

public: /* Special features unique to DistanceField collision */

  // Set what method to use.  Mainly for debugging.
  void getMethods(std::vector<std::string>& methods) const;
  void setMethod(const std::string& method);

public: /* DEBUGGING functions */

  // return the collision spheres for a link in link's collision geometry frame.
  void getLinkSpheres(const std::string& link_name,
                      EigenSTL::vector_Vector3d& centers,
                      std::vector<double>& radii) const;

  // This is just like a call to checkSelfCollision() but all contacts are
  // returned in df_contacts.
  //
  // if req.contacts is true then up to req.max_contacts will be returned.
  // Otherwise up to 1000 will be returned.
  void getSelfCollisionContacts(
                      const CollisionRequest &req,
                      CollisionResult &res,
                      const robot_state::RobotState &state,
                      const AllowedCollisionMatrix *acm,
                      std::vector<DFContact>* df_contacts) const;

  // return the static distance field associated with a link.  Only valid as
  // long as the CollisionRobotDistanceField exists and does not have its field
  // regenerated.
  const StaticDistanceField* getStaticDistanceField(
                      const std::string& link_name) const;

  // Get a list of points showing the distance field.
  // All points between min_dist and max_dist from the surface of
  // the link are returned.
  //
  // If resolution_relative is true then min_dist/max_dist are a multiple of
  // the distance field resolution.  Otherwise they are in meters.
  //
  // Points are in link collision frame.
  void getStaticDistanceFieldPoints(
                      const std::string& link_name,
                      EigenSTL::vector_Vector3d& points,
                      double min_dist = -1.0,
                      double max_dist = 0.0,
                      bool resolution_relative = true) const;


protected:

  virtual void updatedPaddingOrScaling(const std::vector<std::string> &links);


  

private:

  // These types are 16 bits to minimize the size of various arrays for
  // performance reasons.  If more than 65536 spheres or links are needed the
  // size can be increased here.
  typedef uint16_t SphereIndex;
  typedef uint16_t LinkIndex;

  // what method to use to check self collisions
  enum Method
  {
    METHOD_SPHERES,
    METHOD_INTRA_DF,
  };

  // per link internal info for IntraDF self collision checking
  struct DFLink
  {
    const std::string* name_;
    LinkIndex index_in_model_;
    LinkIndex index_in_link_order_;
    SphereIndex sphere_idx_begin_;
    SphereIndex sphere_idx_end_;

    // one bit per link in link_order_.  1=ignore collisions, 0=check collisions
    BitArray acm_bits_;
    
    StaticDistanceField df_;
  };

  //###########################################################################
  //############################### WORK AREA #################################
  //###########################################################################

  /// Contains temporary data used during a collision query.
  // Instead of allocating this every query, a single instance is allocated per
  // thread and used for all queries from that thread.
  struct WorkArea
  {
  public:
    ~WorkArea();


  public:
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

    // if this is non-NULL it will get ALL contacts detected
    std::vector<DFContact>* df_contacts_;
  };


  // get a mutable per-thread work area to store temporary values
  WorkArea& getWorkArea() const;

  //###########################################################################
  //############################### ACCESSOR CONVENIENCE FUNCTIONS ############
  //###########################################################################

  // return the LinkModel for a link index
  const robot_model::LinkModel* linkIndexToLinkModel(int link_index) const;

  // return the LinkState for a link index
  robot_state::LinkState* linkIndexToLinkState(int link_index, const robot_state::RobotState* state) const;

  // return name for a link index
  const std::string& linkIndexToName(int link_index) const;




  // return the link_index of link owning sphere with sphere_index
  int sphereIndexToLinkIndex(int sphere_index) const;

  // return the LinkModel for a particular sphere index in sphere_centers_, sphere_radii_, or sphere_link_map_
  const robot_model::LinkModel* sphereIndexToLinkModel(int sphere_index) const;

  // lookup index for a link. 
  //   -1 if link has no geometry or does not exist.
  const int linkNameToIndex(const std::string& link_name) const;

  // find a link's collision spheres in the srdf
  const srdf::Model::LinkSpheres *getSrdfLinkSpheres(const std::string& link_name) const;

  // Look up DFLink by link name
  const DFLink *linkNameToDFLink(const std::string& link_name) const;
  DFLink       *linkNameToDFLink(const std::string& link_name);

  //###########################################################################
  //############################### INITIALIZATION ############################
  //###########################################################################

  // initialize everything.  Called from constructors.
  void initialize();

  // initialize parameters that affect collision checking.
  // Parameters affect generation of spheres and distance fields, so this must
  // be called before initSpheres() and initLinkDF().
  void initParams();

  //###########################################################################
  //############################### GENERAL ###################################
  //###########################################################################

  // common collision checking function.  Decides what method to use.
  void checkSelfCollision(WorkArea& work) const;

  // true if this link pair should never be checked for collision because it
  // appears in an SRDF DisabledCollisionPair.
  bool never_check_link_pair(const DFLink *link_a, const DFLink *link_b) const;

  // accumulate distance into work.res_.distance
  void setCloseDistance(WorkArea& work, double distance) const;

  // initialize query
  void initQuery(WorkArea& work,
                 const char* descrip,
                 const CollisionRequest *req,
                 CollisionResult *res,
                 const robot_state::RobotState *state1,
                 const robot_state::RobotState *state2,
                 const CollisionRobot *other_robot,
                 const robot_state::RobotState *other_state1,
                 const robot_state::RobotState *other_state2,
                 const AllowedCollisionMatrix *acm) const;

  // show info about query on logInform
  void dumpQuery(const WorkArea& work, const char *descrip) const;

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

  // check self collision using individual static distance fields
  void checkSelfCollisionUsingIntraDF(WorkArea& work) const;

  // Does the work for checkSelfCollisionUsingIntraDF()
  //   work - workarea and description of query
  //   link_list - which links to check
  void checkSelfCollisionUsingIntraDFLoop(WorkArea& work,
                                          const DFLink * const *link_list) const;

  // initialize IntraDF data structures
  void initLinkDF();


  //###########################################################################
  //############################### DATA ######################################
  //###########################################################################

  //===========================================================================
  // Configuration - set in params.cpp
  //===========================================================================

  // This is used to decide how big to make the distance fields.
  // It is roughly the max distance that will be returned by distance*() queries.
  double MAX_DISTANCE_;

  // This is the resolution used for self collision detection.  The accuracy of
  // collision checks will be this plus the tolerance used to generate
  // collision spheres.
  double SELF_COLLISION_RESOLUTION_;

  // what method to use for collision detection.
  // TODO: replace this with a heuristic and/or make it configurable.
  Method method_;

  //===========================================================================
  // Work area
  //===========================================================================
  
  // mutable thread specific work area
  mutable boost::thread_specific_ptr<WorkArea> work_area_;

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
  // DF data for self collision
  //===========================================================================
  
  // This is how much bigger distance fields must be compared to the objects they describe.
  // This is also used as the max distance when calculating distance fields on the fly.
  // The value is based on MAX_DISTANCE_ and the size of the largest link.
  double max_df_distance_;  

  std::vector<DFLink> links_;

  // NULL terminated list of pointers, one for each link in links_
  std::vector<DFLink*> all_links_;
};

}

#endif
