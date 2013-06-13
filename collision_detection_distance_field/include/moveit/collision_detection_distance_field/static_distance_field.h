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

#ifndef MOVEIT_COLLISION_DETECTION_DISTANCE_FIELD__STATIC_DISTANCE_FIELD
#define MOVEIT_COLLISION_DETECTION_DISTANCE_FIELD__STATIC_DISTANCE_FIELD

#include <moveit/distance_field/voxel_grid.h>
#include <Eigen/Geometry>

namespace bodies
{
class Body;
}

namespace collision_detection
{

// This is just a VoxelGrid with some extra methods.
template <typename T>
class VoxelCellGrid : public distance_field::VoxelGrid<T>
{
public:
  VoxelCellGrid();
  virtual ~VoxelCellGrid();

  // Return the data given a point in world coords.
  const T& operator()(const Eigen::Vector3d& point) const;
  using distance_field::VoxelGrid<T>::operator();

  // Get the id of a cell given its position.
  int getCellId(int x, int y, int z) const;

  // get position given a cell id
  void getCellPosition(int cell_id, int& x, int& y, int& z) const;

  // get position given a cell id
  void getCellPosition(int cell_id, double& x, double& y, double& z) const;

  // get position given a cell id
  void getCellPosition(int cell_id, Eigen::Vector3d& point) const;
};


// A voxel entry with a distance and position.
struct DistPosEntry
{
  float distance_;  // distance to nearest occupied cell.
  int cell_id_;     // nearest occupied cell.
};



// A static distance field is a distance field that is expensive to update.
class StaticDistanceField : public VoxelCellGrid<DistPosEntry>
{
public:
  // initialize the field.  Frees old memory, allocates new memory, and calculates DistPosEntry at each cell.
  //   body - describes the occupied cells
  //   resolution - the distance field resolution
  //   space_around_body - how much extra space to include in the distance field outside the body.
  void initialize(const bodies::Body& body,
                  double resolution,
                  double space_around_body);
};

}


template <typename T>
inline collision_detection::VoxelCellGrid<T>::VoxelCellGrid()
{}

template <typename T>
inline collision_detection::VoxelCellGrid<T>::~VoxelCellGrid()
{}


template <typename T>
inline const T& collision_detection::VoxelCellGrid<T>::operator()(const Eigen::Vector3d& point) const
{
  this->operator()(point.x(), point.y(), point.z());
}

template <typename T>
inline int collision_detection::VoxelCellGrid<T>::getCellId(int x, int y, int z) const
{
  return distance_field::VoxelGrid<T>::ref(x,y,z);
}

template <typename T>
inline void collision_detection::VoxelCellGrid<T>::getCellPosition(int cell_id, int& x, int& y, int& z) const
{
  x = cell_id / distance_field::VoxelGrid<T>::stride1_;
  int tmp1 = cell_id % distance_field::VoxelGrid<T>::stride1_;
  y = tmp1 / distance_field::VoxelGrid<T>::stride2_;
  z = tmp1 % distance_field::VoxelGrid<T>::stride2_;
}

template <typename T>
inline void collision_detection::VoxelCellGrid<T>::getCellPosition(int cell_id, double& x, double& y, double& z) const
{
  int xi, yi, zi;
  getCellPosition(cell_id, xi, yi, zi);
  distance_field::VoxelGrid<T>::gridToWorld(xi, yi, xi, x,y,z);
}

template <typename T>
inline void collision_detection::VoxelCellGrid<T>::getCellPosition(int cell_id, Eigen::Vector3d& point) const
{
  getCellPosition(cell_id, point.x(), point.y(), point.z());
}



#endif
