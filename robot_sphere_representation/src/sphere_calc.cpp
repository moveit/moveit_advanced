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

#include <moveit/robot_sphere_representation/sphere_calc.h>
#include <moveit/distance_field/propagation_distance_field.h>
#include <moveit/distance_field/distance_field_common.h>
#include <moveit/robot_state/robot_state.h>
#include <geometric_shapes/bodies.h>
#include <geometric_shapes/body_operations.h>
#include <boost/bind.hpp>
#include <limits>
#include <cmath>
#include <algorithm>
#include <exception>




class ProfTracker
{
public:
  // :.!sed -n 's|^ *PROF_PUSH[_SCOPED]*(\([^)]*\)).*|    X(\1) \\|p' %
  #define PROF_TRACKER_CATEGORIES() \
    X(findClusters) \
    X(SphereCalc_Constructor) \
    X(SphereCalc_thinInternalPoints) \
    X(SphereCalc_thinInternalPoints_make_debug_sets) \
    X(SphereCalc_getOptionalPointSet) \
    X(SphereCalc_getThinnedPointSet) \
    X(SphereCalc_setParams) \
    X(SphereCalc_findSpheres) \
    X(SphereCalc_findSpheres_COMBINED) \
    X(SphereCalc_solveUsingGreedy) \
    X(SphereCalc_eliminateUselessSpheres) \
    X(SphereCalc_solveUsingGradientDescent) \
    X(SphereCalc_solveUsingGobble) \
    X(SphereCalc_createDistanceField) \
    X(SphereCalc_createDistanceField_GatherPoints) \
    X(SphereCalc_createDistanceField_CreateDF) \
    X(SphereCalc_createDistanceField_CreateVoxelGrid) \
    X(SphereCalc_solveUsingClustering) \
    X(SphereCalc_calcQuality) \
    X(SphereCalc_calcQuality_BadCount) \
    X(SphereCalc_calcQuality_MaxDist) \
    X(SphereCalc_calcQuality_Radius) \
    X(SphereCalc_checkQuality) \
    X(SphereCalc_findRadius2) \
    X(SphereCalc_findRadius1) \
    X(RemoveChildOccludedLinks) \
    X(CullLinks) \
    X(GenerateFinalPoints) \
    X(RemoveTempPoints) \
    X(FindLinkPoints) \

  enum Category {
    #define X(x) x,
    PROF_TRACKER_CATEGORIES()
    #undef X
    COUNT
  };

  ProfTracker()
  {
    clear();
  }

  ~ProfTracker()
  {
    print();
  }

  void push(Category cat);
  void pop();
  void pop(Category cat);
  void print();
  void clear();

private:
  void pop(const ros::Time& now);
  void popNoCheck(const ros::Time& now);
  const char* catString(Category cat);

  struct Bucket
  {
    ros::Duration total_;
    std::size_t begin_cnt_;
    bool running_;
    int order_;
    int min_depth_;
  };

  struct StackEntry
  {
    ros::Time begin_;
    Category cat_;
  };
  
  Bucket buckets_[COUNT];
  std::vector<StackEntry> stack_;
  std::vector<Category>order_list_;
};

void ProfTracker::push(Category cat)
{
  ros::Time now = ros::Time::now();

  if (buckets_[cat].running_)
  {
    logInform("ProfTracker: Double push of cat %s\n",catString(cat));
    while (buckets_[cat].running_)
      popNoCheck(now);
  }
  else if (buckets_[cat].order_ < 0)
  {
    buckets_[cat].order_ = order_list_.size();
    order_list_.push_back(cat);
  }

  int depth = stack_.size();
  buckets_[cat].min_depth_ = std::min(buckets_[cat].min_depth_, depth);

  StackEntry entry;
  entry.begin_ = now;
  entry.cat_ = cat;
  stack_.push_back(entry);

  buckets_[cat].running_ = true;
  ++buckets_[cat].begin_cnt_;
}

void ProfTracker::pop()
{
  if (!stack_.empty())
    popNoCheck(ros::Time::now());
}

void ProfTracker::pop(Category cat)
{
  ros::Time now = ros::Time::now();
  while (buckets_[cat].running_)
    popNoCheck(now);
}

void ProfTracker::pop(const ros::Time& now)
{
  if (!stack_.empty())
    popNoCheck(now);
}

void ProfTracker::popNoCheck(const ros::Time& now)
{
  StackEntry &entry = stack_.back();
  buckets_[entry.cat_].total_ += now - entry.begin_;
  buckets_[entry.cat_].running_ = false;
  stack_.pop_back();
}

void ProfTracker::print()
{
  while (!stack_.empty())
    popNoCheck(ros::Time::now());

  logInform("DF2 Profiler:");
  for (std::vector<Category>::iterator it = order_list_.begin() ; it != order_list_.end() ; ++it)
  {
    logInform("  %11.6f s  %6d calls  %*s%s",
      buckets_[*it].total_.toSec(),
      buckets_[*it].begin_cnt_,
      buckets_[*it].min_depth_*2,"",
      catString(Category(*it)));
  }
}

void ProfTracker::clear()
{
  stack_.clear();
  for (int cat = 0 ; cat < COUNT ; ++cat)
  {
    buckets_[cat].total_.sec = 0;
    buckets_[cat].total_.nsec = 0;
    buckets_[cat].begin_cnt_ = 0;
    buckets_[cat].running_ = false;
    buckets_[cat].order_ = -1;
    buckets_[cat].min_depth_ = 100;
  }
  order_list_.clear();
}

const char* ProfTracker::catString(Category cat)
{
  switch(cat)
  {
    #define X(x) case x: return #x;
    PROF_TRACKER_CATEGORIES()
    #undef X
  default:
    return "UNKNOWN";
  }
}


class ProfTrackerEntry
{
public:
  ProfTrackerEntry(ProfTracker& tracker, ProfTracker::Category cat) :
    tracker_(tracker),
    cat_(cat)
  {
    tracker_.push(cat_);
  }

  ~ProfTrackerEntry()
  {
    pop();
  }

  void pop()
  {
    if (cat_ != ProfTracker::COUNT)
    {
      tracker_.pop(cat_);
      cat_ = ProfTracker::COUNT;
    }
  }
private:
  ProfTracker& tracker_;
  ProfTracker::Category cat_;
};

ProfTracker *g_profTracker = 0;

#define PROF_PUSH_SCOPED(cat) ProfTrackerEntry entry_##cat(*g_profTracker, ProfTracker::cat)
#define PROF_PUSH(cat) g_profTracker->push(ProfTracker::cat)
#define PROF_POP() g_profTracker->pop()
#define PROF_PRINT_CLEAR() do { g_profTracker->print(); g_profTracker->clear(); } while(0)











robot_sphere_representation::PointCluster::PointCluster(std::size_t nclusters, const EigenSTL::vector_Vector3d& points) :
  points_(points),
  nclusters_(std::min(nclusters, points.size()))
{
  findClusters();
}

EigenSTL::vector_Vector3d robot_sphere_representation::PointCluster::getClusterPoints(std::size_t cluster_idx)
{
  EigenSTL::vector_Vector3d points;
  if (cluster_idx >= nclusters_)
    return points;

  points.resize(clusters_[cluster_idx].size());
  for (int i = 0 ; i < clusters_[cluster_idx].size() ; ++i)
    points.push_back(points_[clusters_[cluster_idx][i]]);

  return points;
}

void robot_sphere_representation::PointCluster::setNClusters(std::size_t nclusters)
{
  if (nclusters_ != nclusters)
  {
    nclusters_ = nclusters;
    findClusters();
  }
}

void robot_sphere_representation::PointCluster::findClusters()
{
  PROF_PUSH_SCOPED(findClusters);

  nclusters_ = std::max(nclusters_, std::size_t(1));
  nclusters_ = std::min(nclusters_, points_.size());

  clusters_.clear();
  if (nclusters_<1)
    return;

  centers_.resize(nclusters_);
  for (std::size_t i = 0; i < nclusters_ ; ++i)
    centers_[i] = points_[i];
  

  assignClusters();
  moveCenters();

  std::vector<std::vector<int> > old_clusters;

  int cnt = 0;
  do
  {
    swap(old_clusters, clusters_);
    assignClusters();
    moveCenters();
    cnt++;
  }
  while(old_clusters != clusters_ && cnt < 1000);
  
  logInform("Found %d clusters in %d iterations", (int)nclusters_, cnt);
  for (std::size_t i = 0; i < nclusters_ ; ++i)
    logInform("center[%d] = (%7.3f, %7.3f, %7.3f)", (int)i, centers_[i].x(), centers_[i].y(), centers_[i].z());
}

void robot_sphere_representation::PointCluster::moveEmptyCenter(int idx)
{
  logInform("DF2 cluster: moving empty center %d",idx);

  std::vector<double> dist;
  dist.resize(points_.size());

  int closest = 0;
  double closest_d = std::numeric_limits<double>::max();

  for (int i = 0; i < points_.size() ; ++i)
  {
    dist[i] = std::numeric_limits<double>::max();
    const Eigen::Vector3d& p = points_[i];
    double d = std::numeric_limits<double>::max();

    for (int j=0; j<nclusters_; i++)
    {
      if (j == idx)
        continue;

      d = std::min(d, (centers_[j] - points_[i]).squaredNorm());
    }

    if (d < closest_d)
    {
      closest_d = d;
      closest = i;
    }
  }

  centers_[idx] = points_[closest];
}

void robot_sphere_representation::PointCluster::moveCenters()
{
  for (int j=0; j<nclusters_; j++)
  {
    if (clusters_[j].empty())
    {
      moveEmptyCenter(j);
    }
    else
    {
      Eigen::Vector3d sum(0,0,0);
      for (int i = 0; i < clusters_[j].size() ; ++i)
        sum += points_[clusters_[j][i]];
      centers_[j] = sum / double(clusters_[j].size());
    }
  }
}

void robot_sphere_representation::PointCluster::assignClusters()
{
  clusters_.clear();
  clusters_.resize(nclusters_);
  for (std::size_t i = 0; i < nclusters_ ; ++i)
    clusters_[i].reserve(points_.size());

  for (int i = 0; i < points_.size() ; ++i)
  {
    const Eigen::Vector3d& p = points_[i];
    int closest = 0;
    double closest_d = std::numeric_limits<double>::max();
    for (int j=0; j<nclusters_; j++)
    {
      double d = (centers_[j] - p).squaredNorm();
      if (d < closest_d)
      {
        closest_d = d;
        closest = j;
      }
    }
    clusters_[closest].push_back(i);
  }
}


struct robot_sphere_representation::SphereCalc::Voxel
{
  Voxel(double distance = 1000.0) :
    distance_(distance)
  {}

  double distance_;
};

class robot_sphere_representation::SphereCalc::Grid : public distance_field::VoxelGrid<Voxel>
{
public:
  Grid(double size_x, double size_y, double size_z, double resolution,
       double origin_x, double origin_y, double origin_z, const Voxel& default_object) :
    distance_field::VoxelGrid<Voxel>(size_x, size_y, size_z, resolution, origin_x, origin_y, origin_z, default_object),
    oo_resolution_(1.0/resolution),
    vorigin_(origin_x, origin_y, origin_z)
  {}

  
  double getDistanceNoCheck(const V3i& point) const;
  double getDistance(const V3i& point) const;
  double getDistanceQuick(const V3& point) const;
  double getDistanceInterp(const V3& point) const;

private:
  double oo_resolution_;
  V3 vorigin_;
};

inline double robot_sphere_representation::SphereCalc::Grid::getDistanceNoCheck(const V3i& point) const
{
  return getCell(point.x(), point.y(), point.z()).distance_;
}

inline double robot_sphere_representation::SphereCalc::Grid::getDistance(const V3i& point) const
{
  if (isCellValid(point.x(), point.y(), point.z()))
    return getCell(point.x(), point.y(), point.z()).distance_;
  else
    return default_object_.distance_;
}

inline double robot_sphere_representation::SphereCalc::Grid::getDistanceQuick(const V3& point) const
{
  int x, y, z;
  if (worldToGrid(point.x(), point.y(), point.z(), x, y, z))
    return getCell(x, y, z).distance_;
  else
    return default_object_.distance_;
}

double robot_sphere_representation::SphereCalc::Grid::getDistanceInterp(const V3& point) const
{
  V3 offset(point - vorigin_);
  offset *= oo_resolution_;

  V3 o_floor;
  o_floor.x() = std::floor(offset.x());
  o_floor.y() = std::floor(offset.y());
  o_floor.z() = std::floor(offset.z());
  
  int x = int(o_floor.x());
  int y = int(o_floor.y());
  int z = int(o_floor.z());

  if (x<0 || x>=num_cells_[distance_field::DIM_X]-1 ||
      y<0 || y>=num_cells_[distance_field::DIM_Y]-1 ||
      z<0 || z>=num_cells_[distance_field::DIM_Z]-1)
    return default_object_.distance_;

  V3 o_pfrac = offset - o_floor;
  V3 o_nfrac = 1.0 - o_pfrac.array();
  
  return
    (getCell(x+0, y+0, z+0).distance_ * o_nfrac.x() *  o_nfrac.y() * o_nfrac.z()) +
    (getCell(x+1, y+0, z+0).distance_ * o_pfrac.x() *  o_nfrac.y() * o_nfrac.z()) +
    (getCell(x+0, y+1, z+0).distance_ * o_nfrac.x() *  o_pfrac.y() * o_nfrac.z()) +
    (getCell(x+1, y+1, z+0).distance_ * o_pfrac.x() *  o_pfrac.y() * o_nfrac.z()) +
    (getCell(x+0, y+0, z+1).distance_ * o_nfrac.x() *  o_nfrac.y() * o_pfrac.z()) +
    (getCell(x+1, y+0, z+1).distance_ * o_pfrac.x() *  o_nfrac.y() * o_pfrac.z()) +
    (getCell(x+0, y+1, z+1).distance_ * o_nfrac.x() *  o_pfrac.y() * o_pfrac.z()) +
    (getCell(x+1, y+1, z+1).distance_ * o_pfrac.x() *  o_pfrac.y() * o_pfrac.z());
  
}


robot_sphere_representation::SphereCalc::SphereCalc(std::size_t nspheres,
                                                    double resolution,
                                                    const EigenSTL::vector_Vector3d& required_points,
                                                    const EigenSTL::vector_Vector3d& optional_points,
                                                    const std::string& name,
                                                    GenMethod gen_method,
                                                    double tolerance,
                                                    QualMethod qual_method) :
  gen_method_(gen_method),
  tolerance_(tolerance),
  radius2_padding_(resolution*0.1),
  nspheres_requested_(nspheres),
  resolution_(resolution),
  oo_resolution_(1.0/resolution),
  required_points_(required_points),
  optional_points_(optional_points),
  use_required_points_(&required_points_),
  save_history_(true),
  name_(name)
{
  current_.qual_method_ = qual_method;

  PROF_PUSH_SCOPED(SphereCalc_Constructor);
  if (use_required_points_->empty())
  {
    gen_method_ = GenMethod::ZERO_SPHERES;
    createEmptyDistanceField();
  }
  else
  {
    createDistanceField();

    thinInternalPoints();
    use_required_points_ = &thinned_required_points_;
  }

  findSpheres();
  PROF_PRINT_CLEAR();
}

namespace robot_sphere_representation
{

// an array of integer directions to nearby points.
class IntDirVector
{
public:
  // generate an array of direction vectors pointing to all neighbors of a point
  // dist is how many neighbors away to include
  // thickness is how many closer ones to include
  IntDirVector(int dist, int thickness = 1)
  {
    if (thickness == 0 || thickness > dist)
      thickness = dist;

    V3i dp;
    for (dp.x() = -dist ; dp.x() <= dist ; ++dp.x())
    {
      bool xedge = dp.x() < -dist + thickness || dp.x() > dist - thickness;
      for (dp.y() = -dist ; dp.y() <= dist ; ++dp.y())
      {
        bool yedge = dp.y() < -dist + thickness || dp.y() > dist - thickness;
        for (dp.z() = -dist ; dp.z() <= dist ; ++dp.z())
        {
          bool zedge = dp.z() < -dist + thickness || dp.z() > dist - thickness;
          if (!xedge && !yedge && !zedge)
            continue;
          dp_.push_back(dp);
        }
      }
    }
    size_ = dp_.size();
  }


  IntDirVector(V3i dist, V3i thickness = V3i(1,1,1))
  {
    for (int i=0; i<3; i++)
    {
      if (thickness(i) == 0 || thickness(i) > dist(i))
        thickness(i) = dist(i);
    }

    V3i dp;
    for (dp.x() = -dist.x() ; dp.x() <= dist.x() ; ++dp.x())
    {
      bool xedge = dp.x() < -dist.x() + thickness.x() || dp.x() > dist.x() - thickness.x();
      for (dp.y() = -dist.y() ; dp.y() <= dist.y() ; ++dp.y())
      {
        bool yedge = dp.y() < -dist.y() + thickness.y() || dp.y() > dist.y() - thickness.y();
        for (dp.z() = -dist.z() ; dp.z() <= dist.z() ; ++dp.z())
        {
          bool zedge = dp.z() < -dist.z() + thickness.z() || dp.z() > dist.z() - thickness.z();
          if (!xedge && !yedge && !zedge)
            continue;
          dp_.push_back(dp);
        }
      }
    }
    size_ = dp_.size();
  }



  V3iList dp_;
  int size_;
};

}

void robot_sphere_representation::SphereCalc::thinInternalPoints()
{
  static robot_sphere_representation::IntDirVector shell1(1,1);

  PROF_PUSH_SCOPED(SphereCalc_thinInternalPoints);

  thinned_point_set_.clear();
  corner_point_set_.clear();
  edge_point_set_.clear();
  face_point_set_.clear();

  V3i body_size(df_body_aabb_.max_ - df_body_aabb_.min_);
  int body_width_min = std::min(std::min(body_size.x(), body_size.y()), body_size.z());
  int body_width_max = std::max(std::max(body_size.x(), body_size.y()), body_size.z());

  // no point in thinning if the object is small
  if (body_width_max < 20)
  {
    thinned_required_points_ = required_points_;
    return;
  }

  V3iList corner_points;
  V3iList edge_points;
  V3iList face_points;

  // find points inside object and near surface
  V3iSet surf_set;
  V3iList surf_list;
  surf_list.reserve(required_points_.size());
  for (V3List::const_iterator p = required_points_.begin() ; p != required_points_.end() ; ++p)
  {
    V3i ip;
    df_->worldToGrid(p->x(), p->y(), p->z(), ip.x(), ip.y(), ip.z());
    double d = voxel_grid_->getDistance(ip);
    if (d >= -3.0)
    {
      surf_set.insert(ip);
      if (d >= -1.5)
        surf_list.push_back(ip);
    }
  }
  
  // check for nearby points
  for (V3iList::const_iterator p = surf_list.begin() ; p != surf_list.end() ; ++p)
  {
    // opposite bits are shifted by 16
    static const uint32_t bits[27] = {
      0x00000001, // ---
      0x00000002, // --0
      0x00000004, // --+
      0x00000008, // -0-
      0x00000010, // -00
      0x00000020, // -0+
      0x00000040, // -+-
      0x00000080, // -+0
      0x00000100, // -++

      0x00000200, // 0--
      0x00000400, // 0-0
      0x00000800, // 0-+
      0x00001000, // 00-
                  // 000
      0x10000000, // 00+
      0x08000000, // 0+-
      0x04000000, // 0+0
      0x02000000, // 0++

      0x01000000, // +--
      0x00800000, // +-0
      0x00400000, // +-+
      0x00200000, // +0-
      0x00100000, // +00
      0x00080000, // +0+
      0x00040000, // ++-
      0x00020000, // ++0
      0x00010000, // +++
    };
    int cnt = 0;         // how many adjacent samples are occupied?
    uint32_t mask = 0;   // are neighbors on opposite sides occupied?
    for (int idx = 0 ; idx < shell1.size_ ; ++idx)
    {
      V3i np = *p + shell1.dp_[idx];
      if (surf_set.find(np) != surf_set.end())
      {
        cnt++;
        mask |= bits[idx];
      }
    }
      
    if (cnt >= 24)
      continue;     // internal point or inside of a curve
    else if (cnt >= 17)
      face_points.push_back(*p);
    else if (mask & (mask << 16))
      edge_points.push_back(*p);
    else
      corner_points.push_back(*p);
  }

  // subsample face by 1/2
  const int face_dsq_min = 6;

  // subsample inner by 1/4
  const int inner_subsample = 4;

  // points we will keep
  V3iSet thinned_points;

  // keep all corner points
  for (V3iList::const_iterator p = corner_points.begin() ; p != corner_points.end() ; ++p)
    thinned_points.insert(*p);

  // subsample edges by 2x
  for (V3iList::const_iterator p = edge_points.begin() ; p != edge_points.end() ; ++p)
  {
    for (int idx = 0 ; idx < shell1.size_ ; ++idx)
    {
      V3i np = *p + shell1.dp_[idx];
      if (thinned_points.find(np) != thinned_points.end())
        goto skip_edge_point;
    }

    thinned_points.insert(*p);

    skip_edge_point:
    continue;
  }

  // subsample faces by 2x
  for (V3iList::const_iterator p = face_points.begin() ; p != face_points.end() ; ++p)
  {
    for (int idx = 0 ; idx < shell1.size_ ; ++idx)
    {
      V3i np = *p + shell1.dp_[idx];
      if (thinned_points.find(np) != thinned_points.end())
        goto skip_face_point;
    }

    thinned_points.insert(*p);

    skip_face_point:
    continue;
  }

  V3i dist;
  for (int i=0; i<3; i++)
  {
    dist(i) = body_size(i) / 5;
    if (dist(i) < 1)
      dist(i) = 1;
    if (dist(i) > 10)
      dist(i) = 10;
  }
  robot_sphere_representation::IntDirVector ishell(dist,dist);

  // subsample internal points by 3x
  for (V3List::const_iterator p = required_points_.begin() ; p != required_points_.end() ; ++p)
  {
    V3i ip;
    df_->worldToGrid(p->x(), p->y(), p->z(), ip.x(), ip.y(), ip.z());

    for (int idx = 0 ; idx < ishell.size_ ; ++idx)
    {
      V3i np = ip + ishell.dp_[idx];
      if (thinned_points.find(np) != thinned_points.end())
        goto skip_internal_point;
    }

    thinned_points.insert(ip);

    skip_internal_point:
    continue;
  }


  // copy to vector
  thinned_required_points_.clear();
  thinned_required_points_.reserve(thinned_points.size());
  for (V3iSet::const_iterator p = thinned_points.begin() ; p != thinned_points.end() ; ++p)
  { 
    V3 wp;
    df_->gridToWorld(p->x(), p->y(), p->z(), wp.x(), wp.y(), wp.z());
    thinned_required_points_.push_back(wp);
  }

  logInform("Thinned from %d to %d required points",
    required_points_.size(),
    thinned_required_points_.size());

  PROF_PUSH_SCOPED(SphereCalc_thinInternalPoints_make_debug_sets);
  for (V3iList::const_iterator p = face_points.begin() ; p != face_points.end() ; ++p)
    face_point_set_.insert(*p);
  for (V3iList::const_iterator p = edge_points.begin() ; p != edge_points.end() ; ++p)
    edge_point_set_.insert(*p);
  for (V3iList::const_iterator p = corner_points.begin() ; p != corner_points.end() ; ++p)
    corner_point_set_.insert(*p);
}

const robot_sphere_representation::SphereCalc::Result* robot_sphere_representation::SphereCalc::getIteration(int iteration) const
{
  if (iteration < 0 || history_.empty())
    return &best_;

  if (iteration < history_.size())
    return &history_[iteration];

  return &history_[history_.size() - 1];
}

const std::vector<double>& robot_sphere_representation::SphereCalc::getSphereRadii(int iteration) const
{
  const Result *r = getIteration(iteration);
  return r->radius2_;
}

const std::vector<double>& robot_sphere_representation::SphereCalc::getSphereInnerRadii(int iteration) const
{
  const Result *r = getIteration(iteration);
  return r->radius1_;
}

const EigenSTL::vector_Vector3d& robot_sphere_representation::SphereCalc::getSphereCenters(int iteration) const
{
  const Result *r = getIteration(iteration);
  return r->centers_;
}

const robot_sphere_representation::V3iSet& robot_sphere_representation::SphereCalc::getOptionalPointSet() const
{
  if (!optional_point_set_.empty() || optional_points_.empty())
    return optional_point_set_;

  PROF_PUSH_SCOPED(SphereCalc_getOptionalPointSet);
  for (V3List::const_iterator it = optional_points_.begin() ; it != optional_points_.end() ; ++it)
  {
    V3i ip;
    df_->worldToGrid(it->x(), it->y(), it->z(), ip.x(), ip.y(), ip.z());
    optional_point_set_.insert(ip);
  }
  for (V3List::const_iterator it = required_points_.begin() ; it != required_points_.end() ; ++it)
  {
    V3i ip;
    df_->worldToGrid(it->x(), it->y(), it->z(), ip.x(), ip.y(), ip.z());
    optional_point_set_.erase(ip);
  }

  PROF_PRINT_CLEAR();
  return optional_point_set_;
}

const robot_sphere_representation::V3iSet& robot_sphere_representation::SphereCalc::getThinnedPointSet() const
{
  if (!thinned_point_set_.empty() || thinned_required_points_.empty())
    return thinned_point_set_;

  PROF_PUSH_SCOPED(SphereCalc_getThinnedPointSet);
  for (V3List::const_iterator it = thinned_required_points_.begin() ; it != thinned_required_points_.end() ; ++it)
  {
    V3i ip;
    df_->worldToGrid(it->x(), it->y(), it->z(), ip.x(), ip.y(), ip.z());
    thinned_point_set_.insert(ip);
  }

  PROF_PRINT_CLEAR();
  return thinned_point_set_;
}

const robot_sphere_representation::V3iSet& robot_sphere_representation::SphereCalc::getCornerPointSet() const
{
  return corner_point_set_;
}

const robot_sphere_representation::V3iSet& robot_sphere_representation::SphereCalc::getEdgePointSet() const
{
  return edge_point_set_;
}

const robot_sphere_representation::V3iSet& robot_sphere_representation::SphereCalc::getFacePointSet() const
{
  return face_point_set_;
}

double robot_sphere_representation::SphereCalc::getQuality(int iteration, QualMethod qual_method) const
{
  const Result *r = getIteration(iteration);

  if (qual_method == r->qual_method_)
    return r->quality_;

  return calcQuality(*r, qual_method);
}

const char* robot_sphere_representation::SphereCalc::getAlgorithm(int iteration) const
{
  const Result *r = getIteration(iteration);
  return r->algorithm_;
}


void robot_sphere_representation::SphereCalc::setParams(std::size_t nspheres,
                                                         GenMethod gen_method,
                                                         double tolerance,
                                                         QualMethod qual_method)
{
  if (nspheres_requested_ != nspheres ||
      tolerance_ != tolerance ||
      gen_method_ != gen_method ||
      current_.qual_method_ != qual_method)
  {
    PROF_PUSH_SCOPED(SphereCalc_setParams);

    if (use_required_points_->empty())
    {
      gen_method_ = GenMethod::ZERO_SPHERES;
    }
    else
    {
      nspheres_requested_ = nspheres;
      tolerance_ = tolerance;
      gen_method_ = gen_method;
      current_.qual_method_ = qual_method;
    }
    findSpheres();
    PROF_PRINT_CLEAR();
  }
}

void robot_sphere_representation::SphereCalc::Result::clear(int nspheres)
{
  nspheres_ = nspheres;
  centers_.clear();
  radius1_.clear();
  radius2_.clear();
  quality_ = std::numeric_limits<double>::max();
  history_index_ = -1;
  algorithm_ = "Cleared";

  if (nspheres_)
  {
    centers_.resize(nspheres_);
    radius1_.resize(nspheres_);
    radius2_.resize(nspheres_);
    for (std::size_t sphere = 0 ; sphere < nspheres_ ; ++sphere)
    {
      // some dummy positions in case it comes up.
      centers_[sphere] = V3(double(sphere)*0.1,0,0);
      radius1_[sphere] = 0.5;
      radius2_[sphere] = 1.0;
    }
  }
}

void robot_sphere_representation::SphereCalc::clear(int nspheres)
{
  history_.clear();

  iterations_since_improvement_ = 0;

  current_.clear(nspheres);
  spheres_.clear();
  spheres_.resize(nspheres);

  best_ = current_;
}

void robot_sphere_representation::SphereCalc::findSpheres()
{
  PROF_PUSH_SCOPED(SphereCalc_findSpheres);

  clear(0);

  if (!gen_method_.isValid())
    gen_method_ = GenMethod::ONE_SPHERE;
  if (!current_.qual_method_.isValid())
    current_.qual_method_ = QualMethod::DEFAULT;

  if (use_required_points_->empty())
    gen_method_ = GenMethod::ZERO_SPHERES;

  logInform("SphereCalc(%s) ======= BEGIN findSpheres N=%d npoints=%d thin=%d opt=%d gen=%d=%s qual=%d=%s",
    getName().c_str(),
    nspheres_requested_,
    required_points_.size(),
    thinned_required_points_.size(),
    optional_points_.size(),
    gen_method_.toValue(),
    gen_method_.toName().c_str(),
    current_.qual_method_.toValue(),
    current_.qual_method_.toName().c_str());

  PROF_PUSH_SCOPED(SphereCalc_findSpheres_COMBINED);

  step_size_ = resolution_;
  use_required_points_ = &thinned_required_points_;
  switch(gen_method_)
  {
  case GenMethod::CLUSTERING:
    // Does not work very well
    solveUsingClustering();
    break;

  case GenMethod::GOBBLE:
    use_required_points_ = &required_points_;
  case GenMethod::THIN_GOBBLE:
    // Does not work very well
    solveUsingClustering();
    solveUsingGobble();
    break;

  case GenMethod::GRADIENT:
    use_required_points_ = &required_points_;
  case GenMethod::THIN_GRADIENT:
    // gradient descent
    solveUsingClustering();
    solveUsingGradientDescent();
    eliminateUselessSpheres();
    break;

  case GenMethod::GREEDY:
    use_required_points_ = &required_points_;
  case GenMethod::THIN_GREEDY:
    // Use as many spheres as necessary with greedy assignment
    solveUsingGreedy();
    break;

  case GenMethod::GREEDY_GRADIENT:
    use_required_points_ = &required_points_;
  case GenMethod::THIN_GREEDY_GRADIENT:
    // Use GREEDY then GRADIENT
    solveUsingGreedy();
    solveUsingGradientDescent();
    eliminateUselessSpheres();
    break;

  case GenMethod::LIMITGREEDY_GRADIENT:
    use_required_points_ = &required_points_;
  case GenMethod::THIN_LIMITGREEDY_GRADIENT:
    // Use GRADIENT but place initial spheres using GREEDY
    solveUsingGreedy(nspheres_requested_);
    solveUsingGradientDescent();
    eliminateUselessSpheres();
    break;

  case GenMethod::ZERO_SPHERES:
    solveUsingZeroSpheres();
    break;

  case GenMethod::ONE_SPHERE:
  default:
  {
    gen_method_ = GenMethod::ONE_SPHERE;
    // Not useful except to show when an error occurred
    int nspheres_requested = nspheres_requested_;
    nspheres_requested_ = 1;
    solveUsingClustering();
    nspheres_requested_ = nspheres_requested;
  }
    break;

  }

  logInform("SphereCalc(%s) ======= END   findSpheres N=%d npoints=%d thin=%d opt=%d gen=%d=%s qual=%d=%s",
    getName().c_str(),
    nspheres_requested_,
    required_points_.size(),
    thinned_required_points_.size(),
    optional_points_.size(),
    gen_method_.toValue(),
    gen_method_.toName().c_str(),
    current_.qual_method_.toValue(),
    current_.qual_method_.toName().c_str());
}

// Greedy algorithm
//
// Sphere0:
// Find largest sphere that will fit in tolerance.
// Move it so it swallows the most points.
//
// Sphere1:
// Find largest sphere that will fit in tolerance with center that is not in any other sphere.
// Move it so it swallows the most unswallowed points.
// Repeat until all points are claimed or we hit max_spheres (-1=no limit).
void robot_sphere_representation::SphereCalc::solveUsingGreedy(int max_spheres)
{
  PROF_PUSH_SCOPED(SphereCalc_solveUsingGreedy);
  clear(0);

  double tolerance = resolution_ * tolerance_;

  V3List unclaimed = required_points_;

  while (!unclaimed.empty())
  {
    // find deepest point within shape
    double biggest_d = 0;
    for (V3List::const_iterator point = unclaimed.begin() ; point != unclaimed.end() ; ++point)
    {
      double d = -voxel_grid_->getDistanceQuick(*point);
      if (d > biggest_d)
        biggest_d = d;
    }

    // find all candidate center points that are that deep or almost that deep
    biggest_d -= resolution_ * 1.5;
    V3List candidates;
    candidates.reserve(unclaimed.size());
    for (V3List::const_iterator point = unclaimed.begin() ; point != unclaimed.end() ; ++point)
    {
      double d = -voxel_grid_->getDistanceQuick(*point);
      if (d >= biggest_d)
        candidates.push_back(*point);
    }

    if (candidates.empty())
    {
      logError("candidate array is empty -- should never happen!");
      candidates = unclaimed;
    }

    // find the candidate that swallows the most unclaimed points
    V3List::const_iterator best_candidate = candidates.end();
    int best_cnt = -1;
    double best_d = 0;
    for (V3List::const_iterator center = candidates.begin() ; center != candidates.end() ; ++center)
    {
      double d = tolerance + -voxel_grid_->getDistanceQuick(*center);
      double dsq = d*d;
      int cnt = 0;
      for (V3List::const_iterator point = unclaimed.begin() ; point != unclaimed.end() ; ++point)
      {
        if ((*center - *point).squaredNorm() <= dsq)
          cnt++;
      }
      
      if (cnt > best_cnt ||
          (cnt == best_cnt && d > best_d))
      {
        best_candidate = center;
        best_cnt = cnt;
        best_d = d;
      }
    }

    current_.centers_.push_back(*best_candidate);
    current_.radius1_.push_back(-voxel_grid_->getDistanceQuick(*best_candidate));
    current_.radius2_.push_back(best_d);
    Sphere sphere;
    sphere.radius1_squared_ = current_.radius1_.back() * current_.radius1_.back();
    sphere.radius2_squared_ = current_.radius2_.back() * current_.radius2_.back();
    sphere.center_exterior_distance_ = 0;
    spheres_.push_back(sphere);
    current_.nspheres_ = current_.centers_.size();

    // find which points are still unclaimed
    V3List still_unclaimed;
    still_unclaimed.reserve(unclaimed.size() - best_cnt);

    double dsq = best_d * best_d;
    for (V3List::const_iterator point = unclaimed.begin() ; point != unclaimed.end() ; ++point)
    {
      if ((*best_candidate - *point).squaredNorm() > dsq)
        still_unclaimed.push_back(*point);
    }

    swap(still_unclaimed, unclaimed);

    if (save_history_)
      saveCurrentState("Greedy", false, save_history_);

    if (max_spheres > 0 && current_.nspheres_ >= max_spheres)
      break;
  }

  if (unclaimed.empty())
    checkQuality("Greedy(end)");
}

void robot_sphere_representation::SphereCalc::eliminateUselessSpheres()
{
  PROF_PUSH_SCOPED(SphereCalc_eliminateUselessSpheres);
  bool eliminated = false;
  std::vector<bool> elim(current_.centers_.size(), false);
  std::multimap<double,int> center_map;

  // sort by which sphere sticks out farthest
  for (std::size_t sphere = 0 ; sphere < current_.nspheres_ ; ++sphere)
  {
    double d = voxel_grid_->getDistanceInterp(current_.centers_[sphere]) + current_.radius2_[sphere];
    center_map.insert(std::pair<double,int>(d, sphere));
  }

  for (std::multimap<double,int>::const_reverse_iterator it = center_map.rbegin() ; it != center_map.rend() ; ++it)
  {
    for (V3List::const_iterator point = use_required_points_->begin() ; point != use_required_points_->end() ; ++point)
    {
      for (std::size_t sphere = 0 ;; ++sphere)
      {
        if (sphere == current_.nspheres_)
          goto do_not_eliminate;

        if (elim[sphere] || sphere == it->second)
          continue;

        double dsq = (current_.centers_[sphere] - *point).squaredNorm();
        if (dsq <= spheres_[sphere].radius2_squared_)
          break;
      }
    }

    // the sphere was not needed.  All points enclosed without it.
    elim[it->second] = true;
    eliminated = true;

    do_not_eliminate:
    continue;
  }

  if (!eliminated)
    return;

  std::vector<Sphere> spheres;
  V3List centers;
  std::vector<double> radius1;
  std::vector<double> radius2;

  for (std::size_t sphere = 0 ; sphere < current_.nspheres_ ; ++sphere)
  {
    if (elim[sphere])
      continue;
    spheres.push_back(spheres_[sphere]);
    centers.push_back(current_.centers_[sphere]);
    radius1.push_back(current_.radius1_[sphere]);
    radius2.push_back(current_.radius2_[sphere]);
  }
  swap(spheres, spheres_);
  swap(centers, current_.centers_);
  swap(radius1, current_.radius1_);
  swap(radius2, current_.radius2_);
  current_.nspheres_ = current_.centers_.size();

  saveCurrentState("eliminateUselessSpheres", true, save_history_);
}

void robot_sphere_representation::SphereCalc::solveUsingGradientDescent()
{
  PROF_PUSH_SCOPED(SphereCalc_solveUsingGradientDescent);
  bool orig_save_history = save_history_;
  save_history_ = false;

  V3List dirs;

  for (int x=-1; x<2; x++)
  for (int y=-1; y<2; y++)
  for (int z=-1; z<2; z++)
  {
    int cnt = (x!=0) + (y!=0) + (z!=0);
    if (cnt != 1)
      continue;
    V3 dir((double)x, (double)y, (double)z);
    dir.normalize();
    dirs.push_back(dir);
  }

  for (int x=-1; x<2; x++)
  for (int y=-1; y<2; y++)
  for (int z=-1; z<2; z++)
  {
    int cnt = (x!=0) + (y!=0) + (z!=0);
    if (cnt < 2)
      continue;
    V3 dir((double)x, (double)y, (double)z);
    dir.normalize();
    dirs.push_back(dir);
  }

  findRadius1();
  findRadius2();
  checkQuality("Gradient0");

  step_size_ = resolution_ * 0.25;

  int iterations = 0;

  for (;;)
  {
    double old_quality = best_.quality_;
    int improve_cnt = 0;

    int dir_first = 0;
    int dir_last = 5;
    for (int dir_style = 0; dir_style < 2 ; ++dir_style)
    {
      for (std::size_t sphere = 0 ; sphere < current_.nspheres_ ; ++sphere)
      {
        V3 orig_center = current_.centers_[sphere];

        for (int d = dir_first ; d <= dir_last ; d++)
        {
          V3 dir = dirs[d] * step_size_;
          current_.centers_[sphere] = orig_center + dir;

          findRadius1();
          findRadius2();
          checkQuality("Gradient");

          if (best_.quality_ < old_quality)
          {
            old_quality = best_.quality_;
            improve_cnt++;
            if (orig_save_history)
            {
              best_.history_index_ = history_.size();
              history_.push_back(best_);
            }
            break;
          }
          else
          {
            current_.centers_[sphere] = orig_center;
          }
        }
      }

      if (improve_cnt)
        break;

      // try diagonal directions if no improvement with axis-aligned directions
      dir_first = 5;
      dir_last = dirs.size() - 1;
    }

    if (!improve_cnt)
    {
      step_size_ *= 0.5;
step_size_ = 0;
      logWarn("SphereCalc(%s) after %d iterations setting step size to %f",
          getName().c_str(), iterations, step_size_);
      if (step_size_ < resolution_/10)
      {
        logWarn("SphereCalc(%s) no improvement after %d iterations with final step size %f",
          getName().c_str(), iterations, step_size_);
        break;
      }
    }

    if (iterations > 1000)
    {
      logWarn("SphereCalc(%s) abort after %d iterations",getName().c_str(), iterations);
      break;
    }

    iterations++;
  }

  save_history_ = orig_save_history;
}

void robot_sphere_representation::SphereCalc::solveUsingGobble()
{
  PROF_PUSH_SCOPED(SphereCalc_solveUsingGobble);
  int iterations = 0;
  for (;;)
  {
    findRadius1();
    findRadius2();
    checkQuality("Gobble");
    if (checkSuccess())
      break;
    assignExclusiveDistantGoodPoints();
    findCenterMotions();
    if (iterations > 1000)
    {
      logWarn("SphereCalc(%s) abort after %d iterations",getName().c_str(), iterations);
      break;
    }
    applyCenterMotions();
    iterations++;
  }
}

void robot_sphere_representation::SphereCalc::createDistanceField()
{
  PROF_PUSH_SCOPED(SphereCalc_createDistanceField);
  PROF_PUSH_SCOPED(SphereCalc_createDistanceField_GatherPoints);

  df_aabb_.clear();
  df_aabb_.add(required_points_);
  df_aabb_.add(optional_points_);

  V3 aabb_size = df_aabb_.max_ - df_aabb_.min_;
  V3 aabb_delta = (aabb_size * 0.5).array() + 2.0*resolution_;
  df_aabb_.min_ -= aabb_delta;
  df_aabb_.max_ += aabb_delta;

  for (int i=0; i<3; i++)
  {
    double v = df_aabb_.min_(i);
    double x = std::floor(v * oo_resolution_);
    df_aabb_.min_(i) = resolution_ * x;
  }

  df_aabb_size_ = df_aabb_.max_ - df_aabb_.min_;

  const double max_dist = (df_aabb_.max_ - df_aabb_.min_).norm();

  EigenSTL::vector_Vector3d points = required_points_;
  std::copy(optional_points_.begin(), optional_points_.end(), std::back_insert_iterator<EigenSTL::vector_Vector3d>(points));

  PROF_POP();
  PROF_PUSH_SCOPED(SphereCalc_createDistanceField_CreateDF);
  df_.reset(new distance_field::PropagationDistanceField(
                  df_aabb_size_.x(),
                  df_aabb_size_.y(),
                  df_aabb_size_.z(),
                  resolution_,
                  df_aabb_.min_.x(),
                  df_aabb_.min_.y(),
                  df_aabb_.min_.z(),
                  max_dist,
                  true));
  xsize_ = df_->getXNumCells();
  ysize_ = df_->getYNumCells();
  zsize_ = df_->getZNumCells();

  logInform("SphereCalc(%s) AABB:",getName().c_str());
  logInform("     (%7.3f, %7.3f, %7.3f) min",
    df_aabb_.min_.x(),
    df_aabb_.min_.y(),
    df_aabb_.min_.z());
  logInform("     (%7.3f, %7.3f, %7.3f) max",
    df_aabb_.max_.x(),
    df_aabb_.max_.y(),
    df_aabb_.max_.z());
  logInform("     (%7.3f, %7.3f, %7.3f) size",
    (df_aabb_.max_-df_aabb_.min_).x(),
    (df_aabb_.max_-df_aabb_.min_).y(),
    (df_aabb_.max_-df_aabb_.min_).z());
  logInform("     (%d, %d, %d)  ncells",
    xsize_,
    ysize_,
    zsize_);

  // set grid_aabb_
  df_->gridToWorld(0,0,0, grid_aabb_.min_.x(), grid_aabb_.min_.y(), grid_aabb_.min_.z());
  df_->gridToWorld(xsize_-1, ysize_-1, zsize_-1, grid_aabb_.max_.x(), grid_aabb_.max_.y(), grid_aabb_.max_.z());
  big_distance_ = (grid_aabb_.max_ - grid_aabb_.min_).norm() + 10;

  logInform("     (%7.3f, %7.3f, %7.3f) grid_aabb_ min",
    grid_aabb_.min_.x(),
    grid_aabb_.min_.y(),
    grid_aabb_.min_.z());
  logInform("     (%7.3f, %7.3f, %7.3f) grid_aabb_ max",
    grid_aabb_.max_.x(),
    grid_aabb_.max_.y(),
    grid_aabb_.max_.z());

  df_->addPointsToField(points);

  PROF_POP();
  PROF_PUSH_SCOPED(SphereCalc_createDistanceField_CreateVoxelGrid);
  Voxel default_voxel(big_distance_);
  voxel_grid_.reset(new Grid(
                  df_aabb_size_.x(),
                  df_aabb_size_.y(),
                  df_aabb_size_.z(),
                  resolution_,
                  df_aabb_.min_.x(),
                  df_aabb_.min_.y(),
                  df_aabb_.min_.z(),
                  default_voxel));

  df_body_aabb_.clear();

  for (int x=0; x<xsize_; ++x)
  for (int y=0; y<ysize_; ++y)
  for (int z=0; z<zsize_; ++z)
  {
    double d = df_->getDistance(x,y,z);

    // distance field has a bias of -1 on internal points
    d = (d <= -1.0) ? (d+1.0) : d;
    voxel_grid_->getCell(x,y,z).distance_ = d;
    if (d <= 0.0)
      df_body_aabb_.add(V3i(x,y,z));
  }
}

void robot_sphere_representation::SphereCalc::createEmptyDistanceField()
{
  V3List dummy_points;
  dummy_points.push_back(V3(0,0,0));

  df_aabb_.clear();
  df_aabb_.add(dummy_points);

  df_aabb_size_ = df_aabb_.max_ - df_aabb_.min_;

  const double max_dist = 1.0;

  df_.reset(new distance_field::PropagationDistanceField(
                  df_aabb_size_.x() + 2 * resolution_,
                  df_aabb_size_.y() + 2 * resolution_,
                  df_aabb_size_.z() + 2 * resolution_,
                  resolution_,
                  df_aabb_.min_.x(),
                  df_aabb_.min_.y(),
                  df_aabb_.min_.z(),
                  1.0,
                  true));
  xsize_ = df_->getXNumCells();
  ysize_ = df_->getYNumCells();
  zsize_ = df_->getZNumCells();

  // set grid_aabb_
  df_->gridToWorld(0,0,0, grid_aabb_.min_.x(), grid_aabb_.min_.y(), grid_aabb_.min_.z());
  df_->gridToWorld(xsize_-1, ysize_-1, zsize_-1, grid_aabb_.max_.x(), grid_aabb_.max_.y(), grid_aabb_.max_.z());
  big_distance_ = 11;

  Voxel default_voxel(big_distance_);
  voxel_grid_.reset(new Grid(
                  df_aabb_size_.x(),
                  df_aabb_size_.y(),
                  df_aabb_size_.z(),
                  resolution_,
                  df_aabb_.min_.x(),
                  df_aabb_.min_.y(),
                  df_aabb_.min_.z(),
                  default_voxel));

  df_body_aabb_.clear();
}

void robot_sphere_representation::SphereCalc::solveUsingZeroSpheres()
{
  clear(0);
  saveCurrentState("ZeroSpheres", true, save_history_);
}

void robot_sphere_representation::SphereCalc::solveUsingClustering()
{
  PROF_PUSH_SCOPED(SphereCalc_solveUsingClustering);

  int nspheres = std::min(nspheres_requested_, required_points_.size());
  nspheres = std::max(1, nspheres);
  PointCluster cluster(nspheres, required_points_);

  current_.clear(nspheres);
  current_.centers_ = cluster.getCenters();

  spheres_.resize(current_.centers_.size());

  findRadius1();
  findRadius2();
  checkQuality("Clustering");
}

double robot_sphere_representation::SphereCalc::calcQuality(const Result& result, QualMethod qual_method) const
{
  PROF_PUSH_SCOPED(SphereCalc_calcQuality);
  switch (qual_method)
  {
  case QualMethod::MAX_DIST:
    return calcQualityByMaxDistance(result);

  case QualMethod::RADIUS:
    return calcQualityByRadius(result);

  case QualMethod::BADCOUNT:
  default:
    return calcQualityByBadCount(result);
  }
}

// callback for calcQuality()
// Check point to see if it is bad, and if so accumulate it into quality
void robot_sphere_representation::SphereCalc::badPointQuality(
              int sphere,
              BadPointMap* bad_points,
              const V3* point,
              const V3i* ipoint) const
{
  double d = voxel_grid_->getDistanceNoCheck(*ipoint);
  if (d > 0.0)
    (*bad_points)[*ipoint] = d;
}

// calculate quality of current state by looking at included bad points.
double robot_sphere_representation::SphereCalc::calcQualityByBadCount(const Result& result) const
{
  PROF_PUSH_SCOPED(SphereCalc_calcQuality_BadCount);

  int last_bad_cnt = 0;
  BadPointMap bad_points;

  for (std::size_t sphere = 0 ; sphere < result.nspheres_ ; ++sphere)
  {
    const_cast<SphereCalc*>(this)->sphereIterate(
                  boost::bind(&SphereCalc::badPointQuality, const_cast<SphereCalc*>(this), sphere, &bad_points, _1, _2),
                  result.centers_[sphere],
                  result.radius1_[sphere] - resolution_,
                  result.radius2_[sphere]);
  }

  double quality = 0.0;
  for (BadPointMap::const_iterator it = bad_points.begin() ; it != bad_points.end() ; ++it)
    quality += std::pow(10000.0,it->second * oo_resolution_);

  return quality;
}

// calculate quality of current state by looking at farthest sphere point from object
double robot_sphere_representation::SphereCalc::calcQualityByMaxDistance(const Result& result) const
{
  PROF_PUSH_SCOPED(SphereCalc_calcQuality_MaxDist);

  double quality = 0.0;
  for (std::size_t sphere = 0 ; sphere < result.nspheres_ ; ++sphere)
  {
    quality = std::max(quality, (result.radius2_[sphere] - result.radius1_[sphere]) * oo_resolution_);
  }
  return quality;
}

// calculate quality of current state by looking at portion of sphere sticking out
double robot_sphere_representation::SphereCalc::calcQualityByRadius(const Result& result) const
{
  PROF_PUSH_SCOPED(SphereCalc_calcQuality_Radius);

  double quality = 0.0;
  for (std::size_t sphere = 0 ; sphere < result.nspheres_ ; ++sphere)
    quality += std::pow(100.0, result.radius2_[sphere] - result.radius1_[sphere]);

  return quality;
}


// find quality of current state.
// Quality is the sum of x for each bad point included in a sphere, where x is
// the square of the distance from the surface.
void robot_sphere_representation::SphereCalc::checkQuality(const char* algorithm)
{
  PROF_PUSH_SCOPED(SphereCalc_checkQuality);

  current_.quality_ = calcQuality(current_, current_.qual_method_);

  if (current_.qual_method_ != best_.qual_method_)
  {
    best_.qual_method_ = current_.qual_method_;
    best_.quality_ = calcQuality(best_, best_.qual_method_);
  }

  const bool better = current_.quality_ < best_.quality_;
  if (better || save_history_)
  {
    if (better)
      iterations_since_improvement_ = 0;

    saveCurrentState(algorithm, better, save_history_);
  }
}

void robot_sphere_representation::SphereCalc::saveCurrentState(
                                                    const char *algorithm,
                                                    bool save_best,
                                                    bool save_history)
{
  if (!save_best && !save_history)
    return;

  current_.algorithm_ = algorithm;
  current_.history_index_ = -1;

  if (save_history_)
  {
    current_.history_index_ = history_.size();
    history_.push_back(current_);
  }

  if (save_best)
    best_ = current_;

  current_.history_index_ = -1;   // current should always have -1 here
}


bool robot_sphere_representation::SphereCalc::checkSuccess()
{
  iterations_since_improvement_++;
  if (iterations_since_improvement_ > 10)
  {
    if (step_size_ < resolution_ * 0.1)
    {
      logInform("SphereCalc(%s) %d iterations since improvement.  Quitting.",
        getName().c_str(),
        iterations_since_improvement_);
      return true;
    }
    else
    {
      step_size_ *= 0.25;
      logInform("SphereCalc(%s) %d iterations since improvement. setting step_size=%f",
        getName().c_str(),
        iterations_since_improvement_,
        step_size_);
      iterations_since_improvement_ = 0;
    }
  }

  for (std::size_t sphere = 0 ; sphere < current_.nspheres_ ; ++sphere)
  {
    if (current_.radius2_[sphere] > current_.radius1_[sphere])
      return false;
  }

  logInform("SphereCalc(%s) SUCCESS!", getName().c_str());
  return true;
}

// for GOBBLE
void robot_sphere_representation::SphereCalc::applyCenterMotions()
{
  for (std::size_t sphere = 0 ; sphere < current_.nspheres_ ; ++sphere)
  {
    current_.centers_[sphere] += spheres_[sphere].motion_;
  }
}

// for GOBBLE
void robot_sphere_representation::SphereCalc::findCenterMotions()
{
  for (std::size_t sphere = 0 ; sphere < current_.nspheres_ ; ++sphere)
  {

    spheres_[sphere].motion_.setZero();

    if (current_.radius1_[sphere] < resolution_)
      logWarn("SphereCalc(%s) Sphere %d has tiny radius1=%f r2=%f",getName().c_str(), sphere, current_.radius1_[sphere], current_.radius2_[sphere]);

    if (spheres_[sphere].exclusive_distant_good_points_.empty())
    {
      continue;
    }
    else
    {
      AABB aabb(spheres_[sphere].exclusive_distant_good_points_);

      spheres_[sphere].motion_ = ((aabb.max_ + aabb.min_) * 0.5) - current_.centers_[sphere];
      if (spheres_[sphere].motion_.squaredNorm() > step_size_ * step_size_)
      {
        spheres_[sphere].motion_.normalize();
        spheres_[sphere].motion_ *= step_size_;
      }
    }
  }
}

// for GOBBLE: assign distant good points that are only in one r2 sphere to that sphere
void robot_sphere_representation::SphereCalc::assignExclusiveDistantGoodPoints()
{
  for (V3List::const_iterator point = distant_good_points_.begin() ; point != distant_good_points_.end() ; ++point)
  {
    std::size_t exclusive_sphere = current_.nspheres_;
    std::size_t closest_sphere = 0;
    double closest_dsq = big_distance_ * big_distance_;

    for (std::size_t sphere = 0 ;; ++sphere)
    {
      if (sphere >= current_.nspheres_)
      {
        if (exclusive_sphere != current_.nspheres_)
        {
          spheres_[exclusive_sphere].exclusive_distant_good_points_.push_back(*point);
        }
        else
        {
          // not inside r2 of any sphere.  Must be on edge.  Assign to closest sphere.
          spheres_[closest_sphere].exclusive_distant_good_points_.push_back(*point);
        }
        break;
      }

      double dsq = (current_.centers_[sphere] - *point).squaredNorm();
      if (dsq <= spheres_[sphere].radius2_squared_)
      {
        if (exclusive_sphere == current_.nspheres_)
          exclusive_sphere = sphere;
        else
          break;
      }
      else if (dsq < closest_dsq)
      {
        closest_dsq = dsq;
        closest_sphere = sphere;
      }
    }
  }
}

// sphere radii that include all required points
void robot_sphere_representation::SphereCalc::findRadius2()
{
  PROF_PUSH_SCOPED(SphereCalc_findRadius2);
  if (0)
    findRadius2BySmallestRadiusChange();
  else
    findRadius2ByLeastDistance();
}

void acorn_breakpoint()
{
  logInform("WTF?");
}

// sphere radii that include all required points
// Assign points so that we minimize the max R2-R1 for all spheres
void robot_sphere_representation::SphereCalc::findRadius2ByLeastDistance()
{
  distant_good_points_.clear();

  typedef std::multimap<double, V3, std::less<double>, Eigen::aligned_allocator<Eigen::Vector3i> > V3MMap;
  V3MMap point_map;

  for (V3List::const_iterator point = use_required_points_->begin() ; point != use_required_points_->end() ; ++point)
  {
    double best_d = big_distance_;

    for (std::size_t sphere = 0 ;; ++sphere)
    {
      if (sphere >= current_.nspheres_)
      {
        point_map.insert(std::pair<double,V3>(best_d, *point));
        break;
      }

      double dsq = (current_.centers_[sphere] - *point).squaredNorm();
      if (dsq <= spheres_[sphere].radius1_squared_)
      {
        break;
      }
      

      double d = std::sqrt(dsq) - current_.radius1_[sphere] + spheres_[sphere].center_exterior_distance_;
      if (d < best_d)
      {
        best_d = d;
      }
    }
  }

  for (V3MMap::const_reverse_iterator point = point_map.rbegin() ; point != point_map.rend() ; ++point)
  {
    std::size_t best_sphere = 0;
    double best_d = big_distance_;

    for (std::size_t sphere = 0 ;; ++sphere)
    {
      if (sphere >= current_.nspheres_)
      {
        double d = best_d + current_.radius1_[best_sphere];
        double dsq = (current_.centers_[best_sphere] - point->second).squaredNorm();
        if (dsq > spheres_[best_sphere].radius2_squared_)
        {
          spheres_[best_sphere].radius2_squared_ = dsq;
        }
        break;
      }

      double dsq = (current_.centers_[sphere] - point->second).squaredNorm();
      if (dsq <= spheres_[sphere].radius2_squared_)
        break;

      double d = std::sqrt(dsq) - current_.radius1_[sphere] + spheres_[sphere].center_exterior_distance_;
      if (d < best_d)
      {
        best_d = d;
        best_sphere = sphere;
      }
    }
  }

  for (std::size_t sphere = 0 ; sphere < current_.nspheres_ ; ++sphere)
  {
    current_.radius2_[sphere] = std::sqrt(spheres_[sphere].radius2_squared_);
    current_.radius2_[sphere] += radius2_padding_;
    spheres_[sphere].radius2_squared_ = current_.radius2_[sphere] * current_.radius2_[sphere];
  }
}


// sphere radii that include all required points
// Assign each point to the sphere which will grow the least by adding that point
void robot_sphere_representation::SphereCalc::findRadius2BySmallestRadiusChange()
{
  distant_good_points_.clear();

  for (V3List::const_iterator point = use_required_points_->begin() ; point != use_required_points_->end() ; ++point)
  {
    std::size_t closest_sphere = 0;
    double closest_d = big_distance_;

    for (std::size_t sphere = 0 ;; ++sphere)
    {
      if (sphere >= current_.nspheres_)
      {
        if (gen_method_ == GenMethod::GOBBLE)
        {
          spheres_[closest_sphere].distant_good_points_.push_back(*point);
          distant_good_points_.push_back(*point);
        }
        double d = closest_d + current_.radius1_[closest_sphere];
        if (d > current_.radius2_[closest_sphere])
        {
          current_.radius2_[closest_sphere] = d;
          spheres_[closest_sphere].radius2_squared_ = d*d;
        }
        break;
      }

      double dsq = (current_.centers_[sphere] - *point).squaredNorm();
      if (dsq <= spheres_[sphere].radius1_squared_)
        break;

      double d = std::sqrt(dsq) - current_.radius1_[sphere];
      if (d < closest_d)
      {
        closest_d = d;
        closest_sphere = sphere;
      }
    }
  }
}

// sphere radii that do not include any bad points
void robot_sphere_representation::SphereCalc::findRadius1()
{
  PROF_PUSH_SCOPED(SphereCalc_findRadius1);
  for (std::size_t sphere = 0 ; sphere < current_.nspheres_ ; ++sphere)
  {
    current_.radius1_[sphere] = findClosestBadPointDistance(current_.centers_[sphere],
                                                   spheres_[sphere].center_exterior_distance_);
    spheres_[sphere].radius1_squared_ = current_.radius1_[sphere] * current_.radius1_[sphere];
    current_.radius2_[sphere] = current_.radius1_[sphere];
    spheres_[sphere].radius2_squared_ = current_.radius2_[sphere] * current_.radius2_[sphere];
    spheres_[sphere].distant_good_points_.clear();
    spheres_[sphere].exclusive_distant_good_points_.clear();
  }
}

// find distance from an interior (good) point to the nearest exterior (bad) point
// Always returns a positive number.  If argument is exterior then return value
// is clamped to resolution/10
double robot_sphere_representation::SphereCalc::findClosestBadPointDistance(
        const Eigen::Vector3d& center,
        double& exterior_distance)
{
  double d = voxel_grid_->getDistanceInterp(center);
  exterior_distance = 0.0;
  if (d<0.0)
    return -d;

  exterior_distance = d;
  return resolution_ * 0.1;
}

#if 0
Eigen::Vector3d robot_sphere_representation::SphereCalc::findClosestBadPoint(
        const Eigen::Vector3d& center)
{
  V3 closest(0,0,0);
  double closest_dsq = big_distance_ * big_distance_;

  for (int x=0; x<xsize_; ++x)
    for (int y=0; y<ysize_; ++y)
      for (int z=0; z<zsize_; ++z)
      {
//logInform("   -- (%3d %3d %3d) d=%7.3f",x,y,z,voxel_grid_->getDistanceNoCheck(x,y,z));
        if (voxel_grid_->getDistanceNoCheck(x,y,z) <= 0.0)
          continue;
        V3 v;
        df_->gridToWorld(x, y, z, v.x(), v.y(), v.z());
        double dsq = (v - center).squaredNorm();
        if (dsq < closest_dsq)
        {
          closest_dsq = dsq;
          closest = v;
//logInform("       -- BEST! dsq=%7.3f  d=%7.3f", dsq, std::sqrt(dsq));
        }
      }

  return closest;
}
#endif

void robot_sphere_representation::SphereCalc::sphereIterate(
        boost::function<void (const V3*, const V3i*)> func,
        const Eigen::Vector3d& center,
        double min_radius,
        double max_radius)
{
  min_radius = std::max(0.0, min_radius);
  max_radius = std::min(max_radius, big_distance_);
  if (max_radius < min_radius)
    return;

  double minsq = min_radius * min_radius;
  double maxsq = max_radius * max_radius;

  V3i imin, imax;

  double rad = max_radius + resolution_;
  V3 radv(rad, rad, rad);
  V3 min = center - radv;
  V3 max = center + radv;
  min = min.array().max(grid_aabb_.min_.array());
  max = max.array().min(grid_aabb_.max_.array());
  
  if (!df_->worldToGrid(min.x(), min.y(), min.z(), imin.x(), imin.y(), imin.z()))
    abort();
  if (!df_->worldToGrid(max.x(), max.y(), max.z(), imax.x(), imax.y(), imax.z()))
    abort();
  V3 base;
  df_->gridToWorld(imin.x(), imin.y(), imin.z(), base.x(), base.y(), base.z());

  V3 v;
  V3i iv;
  for (iv.x() = imin.x(), v.x() = base.x() ; iv.x()<=imax.x() ; ++iv.x(), v.x()+=resolution_)
    for (iv.y() = imin.y(), v.y() = base.y() ; iv.y()<=imax.y() ; ++iv.y(), v.y()+=resolution_)
      for (iv.z() = imin.z(), v.z() = base.z() ; iv.z()<=imax.z() ; ++iv.z(), v.z()+=resolution_)
      {
        double dsq = (v - center).squaredNorm();
        if (minsq <= dsq && dsq <= maxsq)
        {
          func(&v, &iv);
        }
      }
}

inline const std::string& robot_sphere_representation::Link::getName() const
{
  return lstate_->getName();
}

const robot_sphere_representation::SphereCalc* robot_sphere_representation::Link::getSphereCalc(
                                                          std::size_t nspheres,
                                                          GenMethod gen_method,
                                                          double tolerance,
                                                          QualMethod qual_method)
{
  if (!sphere_calc_)
  {
    V3iSet optional_ipoints;
    std::set_difference(all_points_.begin(),
                        all_points_.end(),
                        final_points_.begin(),
                        final_points_.end(),
                        std::inserter(optional_ipoints, optional_ipoints.end()),
                        robot_sphere_representation::V3iLess());

    logInform("SphereCalc(%s) all_points:%d final_points:%d optional:%d",
      getName().c_str(),
      all_points_.size(),
      final_points_.size(),
      optional_ipoints.size());

    V3List points;
    points.resize(final_points_.size());
    int i = 0;
    for (V3iSet::const_iterator it = final_points_.begin() ; it != final_points_.end() ; ++it, ++i)
    {
      points[i].x() = robot_->gridToWorld(it->x());
      points[i].y() = robot_->gridToWorld(it->y());
      points[i].z() = robot_->gridToWorld(it->z());
    }

    V3List optional_points;
    optional_points.resize(optional_ipoints.size());
    i = 0;
    for (V3iSet::const_iterator it = optional_ipoints.begin() ; it != optional_ipoints.end() ; ++it, ++i)
    {
      optional_points[i].x() = robot_->gridToWorld(it->x());
      optional_points[i].y() = robot_->gridToWorld(it->y());
      optional_points[i].z() = robot_->gridToWorld(it->z());
    }

    logInform("SphereCalc(%s) points:%d optional_points:%d",
      getName().c_str(),
      points.size(),
      optional_points.size());

    sphere_calc_ = new SphereCalc(nspheres, robot_->resolution_, points, optional_points, getName(), gen_method, tolerance, qual_method);
  }
  else
  {
    sphere_calc_->setParams(nspheres, gen_method, tolerance, qual_method);
  }

  return sphere_calc_;
}

const robot_sphere_representation::SphereCalc* robot_sphere_representation::Robot::getLinkSphereCalc(
          const std::string& link_name,
          std::size_t nspheres,
          GenMethod gen_method,
          double tolerance,
          QualMethod qual_method)
{
  Link *link = getLink(link_name);
  if (!link)
    return NULL;

  return link->getSphereCalc(nspheres, gen_method, tolerance, qual_method);
}




void robot_sphere_representation::Link::clusterPoints(std::size_t nclusters)
{
  EigenSTL::vector_Vector3d points;
  points.resize(final_points_.size());
  int i = 0;
  for (V3iSet::const_iterator it = final_points_.begin() ; it != final_points_.end() ; ++it, ++i)
    robot_->gridToWorld(*it, points[i]);

  delete cluster_;
  cluster_ = new robot_sphere_representation::PointCluster(nclusters, points);
} 

EigenSTL::vector_Vector3d robot_sphere_representation::Link::getClusterPoints(
                                                          std::size_t nclusters,
                                                          std::size_t cluster_idx)
{
  if (!cluster_)
    clusterPoints(nclusters);

  cluster_->setNClusters(nclusters);

  return cluster_->getClusterPoints(cluster_idx);
} 

void robot_sphere_representation::Robot::getLinkClusterPointsMarker(
          const std::string& link_name,
          std::size_t nclusters,
          std::size_t cluster_idx,
          visualization_msgs::Marker& m,
          const std::string& frame_id,
          const Eigen::Vector4d& color,
          const std::string ns,
          int id)
{
  initMarker(m, frame_id, color, ns, id);
  if (m.ns.empty())
  {
    std::stringstream ss;
    ss << "LinkPointCluster " << link_name << " (" << cluster_idx << "/" << nclusters << ")";
    m.ns = ss.str();
  }

  Link *link = getLink(link_name);
  if (!link)
    return;

  EigenSTL::vector_Vector3d points = link->getClusterPoints(nclusters, cluster_idx);

  for (EigenSTL::vector_Vector3d::const_iterator it = points.begin() ;
       it != points.end() ;
       ++it)
  {
    geometry_msgs::Point point;
    point.x = it->x();
    point.y = it->y();
    point.z = it->z();
    m.points.push_back(point);
  }
}

#if 0
EigenSTL::vector_Vector3d robot_sphere_representation::Robot::getCluster(
              const std::string& link_name,
              std::size_t nclusters,
              std::size_t cluster_idx)
{
  Link *link = getLink(link_name);
  if (!link)
    return EigenSTL::vector_Vector3d();

  return link->getClusterPoints(nclusters, cluster_idx);
}
#endif

robot_sphere_representation::Link::Link(Robot *robot,
                                        Link *parent,
                                        const robot_model::LinkModel *lmodel)
  : robot_(robot)
  , parent_(parent)
  , lstate_(robot->kstate_->getLinkState(lmodel->getName()))
  , has_collision_(false)
  , cluster_(NULL)
  , sphere_calc_(NULL)
  , body_(NULL)
{}

robot_sphere_representation::Link::~Link()
{
  delete body_;
  delete cluster_;
  delete sphere_calc_;
}

void robot_sphere_representation::Link::calculatePoints()
{
  const shapes::ShapeConstPtr& shape = lstate_->getLinkModel()->getShape();
  if (!shape)
    return;
  body_ = bodies::createBodyFromShape(shape.get());
  if (!body_)
    return;

  body_->setPose(lstate_->getGlobalCollisionBodyTransform());
  EigenSTL::vector_Vector3d points = distance_field::determineCollisionPoints(body_, robot_->resolution_);
  for (EigenSTL::vector_Vector3d::const_iterator it = points.begin() ;
       it != points.end() ;
       ++it)
  {
    V3i ip;
    robot_->worldToGrid(*it, ip);
    all_points_.insert(ip);
  }

  has_collision_ = !all_points_.empty();
} 

Eigen::Vector3d robot_sphere_representation::Link::transformRobotToLink(
      Eigen::Vector3d p)
{
  return lstate_->getGlobalCollisionBodyTransform().inverse() * p;
}

void robot_sphere_representation::Link::transformRobotToLink(
      EigenSTL::vector_Vector3d::iterator begin,
      EigenSTL::vector_Vector3d::iterator end)
{
  Eigen::Affine3d xform = lstate_->getGlobalCollisionBodyTransform().inverse();
  for( ; begin != end ; ++begin)
  {
    *begin = xform * *begin;
  }
}





inline double robot_sphere_representation::Robot::gridToWorld(int grid) const
{
  return resolution_ * grid;
}

inline int robot_sphere_representation::Robot::worldToGrid(double world) const
{
  return int(floor(world * oo_resolution_ + 0.5));
}

inline void robot_sphere_representation::Robot::gridToWorld(const V3i& grid, Eigen::Vector3d& world) const
{
  world.x() = gridToWorld(grid.x());
  world.y() = gridToWorld(grid.y());
  world.z() = gridToWorld(grid.z());
}

inline void robot_sphere_representation::Robot::worldToGrid(const Eigen::Vector3d& world, V3i& grid) const
{
  grid.x() = worldToGrid(world.x());
  grid.y() = worldToGrid(world.y());
  grid.z() = worldToGrid(world.z());
}

void robot_sphere_representation::Robot::initMarker(
          visualization_msgs::Marker& m,
          const std::string& frame_id,
          const Eigen::Vector4d& color,
          const std::string ns,
          int id) const
{
  static int uid = 1;
  if (id == -1)
    id = uid++;

  m.header.stamp = ros::Time::now();
  m.header.frame_id = frame_id;
  m.ns = ns;
  m.id = id;
  m.action = visualization_msgs::Marker::ADD;
  m.type = visualization_msgs::Marker::POINTS;
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = color.x();
  m.color.g = color.y();
  m.color.b = color.z();
  m.color.a = color.w();
  m.lifetime = ros::Duration();

  m.colors.clear();
  m.points.clear();
}

void robot_sphere_representation::Robot::getLinkAllPointsMarker(
          const std::string& link_name,
          visualization_msgs::Marker& m,
          const std::string& frame_id,
          const Eigen::Vector4d& color,
          const std::string ns,
          int id) const
{
  initMarker(m, frame_id, color, ns, id);
  if (m.ns.empty())
  {
    std::stringstream ss;
    ss << "AllLinkPoints " << link_name;
    m.ns = ss.str();
  }

  const Link *link = getLink(link_name);
  if (!link)
    return;

  for (V3iSet::const_iterator it = link->all_points_.begin() ;
       it != link->all_points_.end() ;
       ++it)
  {
    geometry_msgs::Point point;
    point.x = gridToWorld(it->x());
    point.y = gridToWorld(it->y());
    point.z = gridToWorld(it->z());
    m.points.push_back(point);
  }
}

void robot_sphere_representation::Robot::getLinkFinalPointsMarker(
          const std::string& link_name,
          visualization_msgs::Marker& m,
          const std::string& frame_id,
          const Eigen::Vector4d& color,
          const std::string ns,
          int id) const
{
  initMarker(m, frame_id, color, ns, id);
  if (m.ns.empty())
  {
    std::stringstream ss;
    ss << "FinalLinkPoints " << link_name;
    m.ns = ss.str();
  }

  const Link *link = getLink(link_name);
  if (!link)
    return;

  for (V3iSet::const_iterator it = link->final_points_.begin() ;
       it != link->final_points_.end() ;
       ++it)
  {
    geometry_msgs::Point point;
    point.x = gridToWorld(it->x());
    point.y = gridToWorld(it->y());
    point.z = gridToWorld(it->z());
    m.points.push_back(point);
  }
}

void robot_sphere_representation::Robot::initLink(Link *parent, const robot_model::LinkModel *lmodel)
{
  Link *link = new Link(this, parent, lmodel);

  link->calculatePoints();
  links_[lmodel->getName()] = link;

  const std::vector<robot_model::JointModel*>& children = lmodel->getChildJointModels();
  for (std::vector<robot_model::JointModel*>::const_iterator it = children.begin() ;
       it != children.end() ;
       ++it)
  {
    initJoint(link, *it);
  }
}

void robot_sphere_representation::Robot::initJoint(Link *parent, const robot_model::JointModel *jmodel)
{
  initLink(parent, jmodel->getChildLinkModel());
}

// subtract b from a.  a, b, and/or c may alias.
static void diff(const robot_sphere_representation::V3iSet& a,
                 const robot_sphere_representation::V3iSet& b,
                 robot_sphere_representation::V3iSet& result)
{
  robot_sphere_representation::V3iSet r;

  std::set_difference(a.begin(),
                      a.end(),
                      b.begin(),
                      b.end(),
                      std::inserter(r, r.end()),
                      robot_sphere_representation::V3iLess());
  swap(result, r);
}


/** Traverse links in the robot in joint tree order.
 *  link_model - first link to traverse
 *  max_depth - how deep to go.  -1 for all.  0 for just this link and no children.
 *  pre - called before traversing to child.  Return true to stop traversal (skip children), true to keep going deeper.
 *  post - called after traversing children.  */
static void TraverseLinkTree(
    const robot_model::LinkModel &link_model,
    int max_depth,
    boost::function<bool (const robot_model::LinkModel&)> pre,
    boost::function<void (const robot_model::LinkModel&)> post)
{
  //const robot_model::LinkModel *link_model = kmodel_.getLinkModel(link_name);
  bool stop = pre(link_model);
  if (!stop && max_depth != 0)
  {
    if (max_depth > 0)
      --max_depth;

    for (std::vector<robot_model::JointModel*>::const_iterator it = link_model.getChildJointModels().begin() ;
         it != link_model.getChildJointModels().end() ;
         ++it)
    {
      const robot_model::LinkModel *child_link = (*it)->getChildLinkModel();
      if (child_link)
        TraverseLinkTree(*child_link, max_depth, pre, post);
    }
  }
  post(link_model);
}

static bool TraverseLinkTreeNullPre(const robot_model::LinkModel&)
{
  return false;
}

static void TraverseLinkTreeNullPost(const robot_model::LinkModel&)
{
}

bool robot_sphere_representation::Robot::RemoveChildOccludedLinks2Pre(const robot_model::LinkModel& link_model,
                                                                   Link *parent)
{
  Link *link = getLink(link_model.getName());
  if (link && link != parent && !link->all_points_with_children_.empty())
  {
    parent->all_points_with_children_.insert(link->all_points_with_children_.begin(),
                                             link->all_points_with_children_.end());
    return true;
  }
  return false;
}

void robot_sphere_representation::Robot::RemoveChildOccludedLinksPost(const robot_model::LinkModel& link_model)
{
  Link *link = getLink(link_model.getName());
  if (link && link->has_collision_)
  {
    TraverseLinkTree(*kmodel_->getRootLink(), 
                     -1,
                     boost::bind(&Robot::RemoveChildOccludedLinks2Pre, this, _1, link),
                     TraverseLinkTreeNullPost);

    V3iSet points;
#if 1
    diff(link->all_points_, link->all_points_with_children_, points);
#else
    std::set_difference(link->all_points_.begin(),
                        link->all_points_.end(),
                        link->all_points_with_children_.begin(),
                        link->all_points_with_children_.end(),
                        std::inserter(points, points.end()),
                        V3iLess());
#endif
    logInform("RemoveChildOccludedLinks link %30s has %4d of %4d points remaining.  %4d with children.",
        link_model.getName().c_str(),
        points.size(),
        link->all_points_.size(),
        link->all_points_with_children_.size());
    if (points.empty())
    {
      link->has_collision_ = false;
      logInform("XXXXXXXXXXX RemoveChildOccludedLinks removing link %s", link_model.getName().c_str());
    }

    link->all_points_with_children_.insert(link->all_points_.begin(),
                                           link->all_points_.end());
  }
}

void robot_sphere_representation::Robot::RemoveChildOccludedLinks()
{
  logInform("BEGIN TRAVERSING LINKS for RemoveChildOccludedLinks");
  PROF_PUSH_SCOPED(RemoveChildOccludedLinks);
  TraverseLinkTree(*kmodel_->getRootLink(), 
                   -1,
                   TraverseLinkTreeNullPre,
                   boost::bind(&Robot::RemoveChildOccludedLinksPost, this, _1));
  logInform("END TRAVERSING LINKS for RemoveChildOccludedLinks");
}

bool robot_sphere_representation::Robot::CullLinks2Pre(const robot_model::LinkModel& link_model,
                                                         Link *parent)
{
  Link *link = getLink(link_model.getName());
  if (link && link != parent && link->has_collision_)
  {
      diff(parent->post_cull_points_, link->all_points_, parent->post_cull_points_);
      logInform("CullLinks link %30s with %d points after removing child %s.",
          parent->getName().c_str(),
          parent->post_cull_points_.size(),
          link_model.getName().c_str());
      if (parent->post_cull_points_.empty())
        return true;
  }
  return false;
}

bool robot_sphere_representation::Robot::CullLinksPre(const robot_model::LinkModel& link_model)
{
  Link *link = getLink(link_model.getName());
  if (link && link->has_collision_)
  {
    link->post_cull_points_ = link->all_points_;
    logInform("CullLinks link %30s with %d points.",
        link_model.getName().c_str(),
        link->all_points_.size());

    int cnt = 0;
    for (Link *parent = link->parent_; cnt<3 && parent; parent = parent->parent_)
    {
      if (!parent->has_collision_ || parent->all_points_.empty())
        continue;

      V3iSet points2;
      
      std::set_difference(link->post_cull_points_.begin(),
                          link->post_cull_points_.end(),
                          parent->all_points_.begin(),
                          parent->all_points_.end(),
                          std::inserter(points2, points2.end()),
                          V3iLess());
      swap(link->post_cull_points_, points2);
      logInform("CullLinks link %30s with %d points after removing parent %s.",
          link_model.getName().c_str(),
          link->all_points_.size(),
          parent->getName().c_str());
      cnt++;
      if (link->post_cull_points_.empty())
        break;
    }
    if (!link->post_cull_points_.empty())
    {
      TraverseLinkTree(link_model,
                       3,
                       boost::bind(&Robot::CullLinks2Pre, this, _1, link),
                       TraverseLinkTreeNullPost);
    }
    logInform("CullLinks link %30s has %4d of %4d points remaining.",
        link_model.getName().c_str(),
        link->post_cull_points_.size(),
        link->all_points_.size());
    if (link->post_cull_points_.empty())
    {
      link->has_collision_ = false;
      logInform("XXXXXXXXXXX CullLinks removing link %s", link_model.getName().c_str());
    }
  }
  return false;
}

void robot_sphere_representation::Robot::CullLinks()
{
  logInform("BEGIN TRAVERSING LINKS for CullLinks");
  PROF_PUSH_SCOPED(CullLinks);
  TraverseLinkTree(*kmodel_->getRootLink(), 
                   -1,
                   boost::bind(&Robot::CullLinksPre, this, _1),
                   TraverseLinkTreeNullPost);
  logInform("END TRAVERSING LINKS for CullLinks");
}


bool robot_sphere_representation::Robot::GenerateFinalPointsPre(const robot_model::LinkModel& link_model)
{
  Link *link = getLink(link_model.getName());
  if (link)
  {
    delete link->cluster_;
    link->cluster_ = NULL;
  }

  if (link && link->has_collision_)
  {
    Link *parent = link->parent_;
    while (parent && !parent->has_collision_)
      parent = parent->parent_;

    link->parent_points_.clear();
    if (parent)
    {
      link->parent_points_ = parent->parent_points_;
      link->parent_points_.insert(parent->all_points_.begin(),
                                  parent->all_points_.end());
    }

#if 1
    diff(link->all_points_, link->parent_points_, link->final_points_);
#else
    std::set_difference(link->all_points_.begin(),
                        link->all_points_.end(),
                        link->parent_points_.begin(),
                        link->parent_points_.end(),
                        std::inserter(link->final_points_, link->final_points_.end()),
                        V3iLess());
#endif

    logInform("GenerateFinalPoints link %30s has %4d of %4d points remaining.  %4d parent points.",
        link_model.getName().c_str(),
        link->final_points_.size(),
        link->all_points_.size(),
        link->parent_points_.size());
    if (link->final_points_.empty())
    {
      link->has_collision_ = false;
      logInform("XXXXXXXXXXX GenerateFinalPointsPre removing link %s", link_model.getName().c_str());
    }
  }
  return false;
}

void robot_sphere_representation::Robot::GenerateFinalPoints()
{
  logInform("BEGIN TRAVERSING LINKS for GenerateFinalPoints");
  PROF_PUSH_SCOPED(GenerateFinalPoints);
  TraverseLinkTree(*kmodel_->getRootLink(), 
                   -1,
                   boost::bind(&Robot::GenerateFinalPointsPre, this, _1),
                   TraverseLinkTreeNullPre);
  logInform("END TRAVERSING LINKS for GenerateFinalPoints");
}



bool robot_sphere_representation::Robot::RemoveTempPointsPre(const robot_model::LinkModel& link_model)
{
  Link *link = getLink(link_model.getName());
  if (link)
  {
    link->all_points_with_children_.clear();
    link->parent_points_.clear();
    link->post_cull_points_.clear();
  }
  return false;
}

void robot_sphere_representation::Robot::RemoveTempPoints()
{
  PROF_PUSH_SCOPED(RemoveTempPoints);
  TraverseLinkTree(*kmodel_->getRootLink(), 
                   -1,
                   boost::bind(&Robot::RemoveTempPointsPre, this, _1),
                   TraverseLinkTreeNullPost);
}


robot_sphere_representation::Robot::Robot(
              const robot_model::RobotModelConstPtr& kmodel,
              double resolution) :
  kmodel_(kmodel),
  kstate_(new robot_state::RobotState(kmodel_)),
  resolution_(resolution),
  oo_resolution_(1.0/resolution)
{
  logInform("Generating CollisionRobotDF2 init data...");
  if (!g_profTracker)
    g_profTracker = new ProfTracker();

  kstate_->setToDefaultValues();

  PROF_PUSH(FindLinkPoints);
  initJoint(NULL, kmodel->getRoot());
  PROF_POP();

#if 0
  RemoveChildOccludedLinks();
#else
  CullLinks();
#endif

  GenerateFinalPoints();
  RemoveTempPoints();
  
  PROF_PRINT_CLEAR();
  logInform("Done generating CollisionRobotDF2 init data...");
}

robot_sphere_representation::Robot::~Robot()
{
  for (std::map<std::string, Link*>::iterator it = links_.begin() ; it != links_.end() ; ++it)
    delete it->second;
}

