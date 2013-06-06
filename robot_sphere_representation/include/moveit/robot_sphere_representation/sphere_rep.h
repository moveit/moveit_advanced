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

#ifndef MOVEIT_ROBOT_SPHERE_REPRESENTATION_SPHERE_REP_
#define MOVEIT_ROBOT_SPHERE_REPRESENTATION_SPHERE_REP_

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>

#include <eigen_stl_containers/eigen_stl_containers.h>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>
#include <set>

#include <visualization_msgs/Marker.h>

#include <moveit/robot_sphere_representation/method_enums.h>


namespace bodies
{
  class Body;
}

namespace distance_field
{
  class PropagationDistanceField;
}

namespace robot_model
{
class RobotModel;
class LinkModel;
class JointModel;
}

namespace robot_state
{
class RobotState;
class LinkState;
}

namespace robot_sphere_representation
{

typedef Eigen::Vector3d V3;
typedef EigenSTL::vector_Vector3d V3List;


typedef Eigen::Vector3i V3i;
typedef std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > V3iList;


/** point comparison for map insertion */
struct V3iLess {
  bool operator()(const V3i& a, const V3i& b) const
  {
    if (a.x() < b.x())
      return true;
    if (a.x() > b.x())
      return false;
    if (a.y() < b.y())
      return true;
    if (a.y() > b.y())
      return false;
    return a.z() < b.z();
  }
};

typedef std::set<V3i, V3iLess, Eigen::aligned_allocator<Eigen::Vector3i> > V3iSet;


template<class T>
class basic_AABB
{
public:
  typedef Eigen::Matrix<T, 3, 1> Vec3;
  typedef std::vector<Vec3, Eigen::aligned_allocator<Vec3> > vector_Vec3;
  basic_AABB()
  {
    clear();
  }

  basic_AABB(const vector_Vec3& points)
  {
    clear();
    add(points);
  }

  void add(const Vec3& point)
  {
    min_ = min_.array().min(point.array());
    max_ = max_.array().max(point.array());
  }

  void add(const vector_Vec3& points)
  {
    for (typename vector_Vec3::const_iterator it = points.begin() ; it != points.end() ; ++it)
      add(*it);
  }

  void clear()
  {
    min_.x() = std::numeric_limits<T>::max();
    min_.y() = std::numeric_limits<T>::max();
    min_.z() = std::numeric_limits<T>::max();
    if (std::numeric_limits<T>::is_integer)
    {
      max_.x() = std::numeric_limits<T>::min();
      max_.y() = std::numeric_limits<T>::min();
      max_.z() = std::numeric_limits<T>::min();
    }
    else
    {
      max_.x() = -std::numeric_limits<T>::max();
      max_.y() = -std::numeric_limits<T>::max();
      max_.z() = -std::numeric_limits<T>::max();
    }
  }

  Vec3 min_;
  Vec3 max_;
};

typedef basic_AABB<double> AABB;
typedef basic_AABB<int> AABBi;



class PointCluster
{
public:
  PointCluster(std::size_t nclusters, const EigenSTL::vector_Vector3d& points);
  void setNClusters(std::size_t nclusters);
  EigenSTL::vector_Vector3d getClusterPoints(std::size_t cluster_idx);
  const EigenSTL::vector_Vector3d& getCenters() const { return centers_; }

private:
  void findClusters();
  void moveEmptyCenter(int idx);
  void moveCenters();
  void assignClusters();

  std::size_t nclusters_;
  std::vector<std::vector<int> > clusters_;

  EigenSTL::vector_Vector3d points_;
  EigenSTL::vector_Vector3d centers_;
};


// for finding a set of N spheres which tightly bound the shape in question.
class SphereRep
{
public:

#if 0
// THESE ARE MOVED TO method_enums.h

  // default is first in list
  #define DF2_SPHEREREP_METHOD_LIST(x) \
          x(THIN_LIMITGREEDY_GRADIENT) \
          x(THIN_GREEDY_GRADIENT) \
          x(THIN_GREEDY) \
          x(THIN_GRADIENT) \
          x(THIN_GOBBLE) \
          x(LIMITGREEDY_GRADIENT) \
          x(GREEDY_GRADIENT) \
          x(GREEDY) \
          x(GRADIENT) \
          x(GOBBLE) \

  enum Method {
    #define X(e) e,
    DF2_SPHEREREP_METHOD_LIST(X)
    METHOD_DEFAULT
    #undef X
  };

  // default is first in list
  #define DF2_SPHEREREP_QUALITY_METHOD_LIST(x) \
          x(QUAL_MAX_DIST) \
          x(QUAL_BADCOUNT) \
          x(QUAL_RADIUS) \

  enum QualityMethod {
    #define X(e) e,
    DF2_SPHEREREP_QUALITY_METHOD_LIST(X)
    QUAL_DEFAULT
    #undef X
  };
#endif

  SphereRep(std::size_t nspheres,
            double resolution,
            const EigenSTL::vector_Vector3d& required_points,
            const EigenSTL::vector_Vector3d& optional_points,
            const std::string& name = "",
            GenMethod method = GenMethod::DEFAULT,
            double tolerance = 1.0,
            QualMethod quality_method = QualMethod::DEFAULT);
  void setParams(
            std::size_t nspheres,
            GenMethod method = GenMethod::DEFAULT,
            double tolerance = 1.0,
            QualMethod quality_method = QualMethod::DEFAULT);

#if 0
  // convert between string and GenMethod/QualMethod
  static const std::vector<std::string>& getMethodNames();
  static GenMethod parseMethodName(const std::string& method);
  static const char* getMethodName(GenMethod method);
  static const std::vector<std::string>& getQualityMethodNames();
  static QualMethod parseQualityMethodName(const std::string& quality_method);
  static const char* getQualityMethodName(QualMethod quality_method);
#endif

  const std::vector<double>& getSphereRadii(int iteration = -1) const;
  const std::vector<double>& getSphereInnerRadii(int iteration = -1) const;
  const EigenSTL::vector_Vector3d& getSphereCenters(int iteration = -1) const;
  double getQuality(int iteration = -1, QualMethod quality_method=QualMethod::DEFAULT) const;
  const char* getAlgorithm(int iteration = -1) const;
  int getBestIteration() const { return best_.history_index_; }
  int getNumIterations() const { return history_.size(); }

  void setName(const std::string& name) { name_ = name; }
  const std::string& getName() const { return name_; }

  const distance_field::PropagationDistanceField* getDistanceField() const { return df_.get(); }

  // optional points are in the link but overlap the parent link
  const V3iSet& getOptionalPointSet() const;

  // thinned points are a subset of the required points - dense along corners and edges, subsampled internally
  // corner, edge, and face points are points thought to be on that part of the object
  const V3iSet& getThinnedPointSet() const;
  const V3iSet& getCornerPointSet() const;
  const V3iSet& getEdgePointSet() const;
  const V3iSet& getFacePointSet() const;

private:
  struct Result;

  const Result* getIteration(int iteration = -1) const;
  void findSpheres();

  void solveUsingGreedy(int max_spheres = -1);
  void solveUsingGradientDescent();
  void solveUsingGobble();
  void solveUsingClustering();
  void eliminateUselessSpheres();

  void checkQuality(const char* algorithm);

  double calcQuality(const Result& result, QualMethod quality_method) const;
  double calcQualityByBadCount(const Result& result) const;
  double calcQualityByRadius(const Result& result) const;
  double calcQualityByMaxDistance(const Result& result) const;
  typedef std::map<V3i, double, V3iLess, Eigen::aligned_allocator<Eigen::Vector3i> > BadPointMap;
  void badPointQuality(int sphere, BadPointMap* bad_points, const V3* point, const V3i* ipoint) const;

  void clear(int nspheres);
  void createDistanceField();
  void thinInternalPoints();
  void saveCurrentState(const char *algorithm, bool saveBest, bool save_history);
  bool checkSuccess();
  void applyCenterMotions();
  void findCenterMotions();
  void findRadius1();
  void findRadius2();
  void findRadius2BySmallestRadiusChange();
  void findRadius2ByLeastDistance();
  void assignExclusiveDistantGoodPoints();

  // distance to closest bad point.  clamped to minimum of resolution/10
  double findClosestBadPointDistance(const Eigen::Vector3d& center, double& exterior_distance);
  Eigen::Vector3d findClosestBadPoint(const Eigen::Vector3d& center);

  void sphereIterate(
        boost::function<void (const V3*, const V3i*)> func,
        const Eigen::Vector3d& center,
        double min_radius,
        double max_radius);


  GenMethod gen_method_;
  std::size_t nspheres_requested_;
  double tolerance_;
  double radius2_padding_;
  double resolution_;
  double oo_resolution_;
  double step_size_;      // how far each sphere center moves each iteration
  int xsize_;
  int ysize_;
  int zsize_;
  AABB df_aabb_;          // entire distance field
  V3 df_aabb_size_;
  AABBi df_body_aabb_;   // just the shape, in grid coords
  boost::shared_ptr<distance_field::PropagationDistanceField> df_;
  

  class Grid;
  class Voxel;
  boost::shared_ptr<Grid> voxel_grid_;
  

  V3List required_points_;
  V3List optional_points_;
  V3List thinned_required_points_;
  V3List *use_required_points_;     // points to required_points_ or thinned_required_points_

  struct Sphere
  {
    double radius1_squared_;
    double radius2_squared_;
    double center_exterior_distance_; // if center is outside shape, distance from shape (else 0)
    V3 motion_;                 // how center should move
    V3List distant_good_points_;  // good points not in any r1 sphere that are closer to this sphere than any other
    V3List exclusive_distant_good_points_; // good points not in any r1 sphere that in my r2 sphere and no other r2 sphere
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  std::vector<Sphere> spheres_;

#define SPHERE_REP_USE_CURRENT 1
#if !SPHERE_REP_USE_CURRENT
  V3List centers_;
  std::vector<double> radius1_;
  std::vector<double> radius2_;
  QualMethod quality_method_;
  std::size_t nspheres_;
  int iteration_;
#endif

  V3List distant_good_points_; // good points not in any r1 sphere


  AABB grid_aabb_;      // the current shape and surrounding area
  double big_distance_; // a big number -- longer than the diagonal of the grid_aabb_

  std::string name_;        // for debug messages
  int iterations_since_improvement_;

  struct Result
  {
    void clear(int nspheres);

    int nspheres_;
    V3List centers_;
    std::vector<double> radius1_;
    std::vector<double> radius2_;
    double quality_;
    QualMethod quality_method_;
    int history_index_;
    const char *algorithm_;
  };
#if SPHERE_REP_USE_CURRENT
  Result current_;
#endif
  Result best_;
  std::vector<Result> history_; // for debugging
  bool save_history_;

  mutable V3iSet optional_point_set_;
  mutable V3iSet thinned_point_set_;
  mutable V3iSet corner_point_set_;
  mutable V3iSet edge_point_set_;
  mutable V3iSet face_point_set_;
};



class Robot;

/** one link in the Robot.  For initializing CollisionRobotDF2 */
class Link {
public:
  Link(Robot *robot, Link *parent, const robot_model::LinkModel *lmodel);
  ~Link();
  const std::string& getName() const;
  void calculatePoints();
  void clusterPoints(std::size_t nclusters);
  EigenSTL::vector_Vector3d getClusterPoints(std::size_t nclusters,
                                             std::size_t cluster_idx);
  const SphereRep* getSphereRep(std::size_t nspheres,
                                GenMethod method = GenMethod::DEFAULT,
                                double tolerance = 1.0,
                                QualMethod quality_method = QualMethod::DEFAULT);

  // transform points from robot frame to link collision frame
  Eigen::Vector3d transformRobotToLink(Eigen::Vector3d p);
  void transformRobotToLink(EigenSTL::vector_Vector3d::iterator begin,
                            EigenSTL::vector_Vector3d::iterator end); 

  bool hasCollision()
  {
    return has_collision_;
  }

private:
  Robot *robot_;
  Link *parent_;
  robot_state::LinkState *lstate_;
  V3iSet all_points_;                 // points in link
  V3iSet post_cull_points_;           // temp: points in link that are not culled
  V3iSet all_points_with_children_;   // temp: points in child links
  V3iSet parent_points_;              // temp: points in all parent links
  V3iSet final_points_;               // points in link with parent points removed
  bool has_collision_;    // worth colliding after culling?
  bodies::Body* body_;
  PointCluster *cluster_;
  SphereRep *sphere_rep_;

  friend class Robot;
};

/** Robot representation.  For initializing CollisionRobotDF2 */
class Robot {
public:
  Robot(const boost::shared_ptr<const robot_model::RobotModel>& kmodel, double resolution);
  ~Robot();

#if 0
  // read/write yaml file
  void writeYaml(std::ostream) const;
  void readYaml(std::istream);
#endif

  void initLink(Link *parent, const robot_model::LinkModel *lmodel);
  void initJoint(Link *parent, const robot_model::JointModel *jmodel);

  double gridToWorld(int grid) const;
  int worldToGrid(double world) const;
  void gridToWorld(const V3i& grid, Eigen::Vector3d& world) const;
  void worldToGrid(const Eigen::Vector3d& world, V3i& grid) const;

  void getLinkAllPointsMarker(
                        const std::string& link_name,
                        visualization_msgs::Marker& m,
                        const std::string& frame_id,
                        const Eigen::Vector4d& color = Eigen::Vector4d(1,1,1,1),
                        const std::string ns = "",
                        int id=0) const;

  void getLinkFinalPointsMarker(
                        const std::string& link_name,
                        visualization_msgs::Marker& m,
                        const std::string& frame_id,
                        const Eigen::Vector4d& color = Eigen::Vector4d(1,1,1,1),
                        const std::string ns = "",
                        int id=0) const;

  void getLinkClusterPointsMarker(
                        const std::string& link_name,
                        std::size_t nclusters,
                        std::size_t cluster_idx,
                        visualization_msgs::Marker& m,
                        const std::string& frame_id,
                        const Eigen::Vector4d& color = Eigen::Vector4d(1,1,1,1),
                        const std::string ns = "",
                        int id=0);
  const SphereRep* getLinkSphereRep(
                        const std::string& link_name,
                        std::size_t nspheres,
                        GenMethod method = GenMethod::DEFAULT,
                        double tolerance = 1.0,
                        QualMethod quality_method = QualMethod::DEFAULT);

  Link *getLink(const std::string& name)
  {
    std::map<std::string, Link*>::const_iterator link_it = links_.find(name);
    return (link_it == links_.end()) ? NULL : link_it->second;
  }
  const Link *getLink(const std::string& name) const
  {
    std::map<std::string, Link*>::const_iterator link_it = links_.find(name);
    return (link_it == links_.end()) ? NULL : link_it->second;
  }

private:
  friend struct Link;

  void initMarker(visualization_msgs::Marker& m,
                  const std::string& frame_id,
                  const Eigen::Vector4d& color,
                  const std::string ns,
                  int id) const;

  bool RemoveChildOccludedLinks2Pre(const robot_model::LinkModel& link_model, Link *parent);
  void RemoveChildOccludedLinksPost(const robot_model::LinkModel& link_model);
  void RemoveChildOccludedLinks();

  bool CullLinks2Pre(const robot_model::LinkModel& link_model, Link *parent);
  bool CullLinksPre(const robot_model::LinkModel& link_model);
  void CullLinks();


  bool GenerateFinalPointsPre(const robot_model::LinkModel& link_model);
  void GenerateFinalPoints();

  bool RemoveTempPointsPre(const robot_model::LinkModel& link_model);
  void RemoveTempPoints();


  boost::shared_ptr<const robot_model::RobotModel> kmodel_;
  boost::shared_ptr<robot_state::RobotState> kstate_;
  
  std::map<std::string, Link*> links_;
  double resolution_;
  double oo_resolution_;
};


}

#endif
