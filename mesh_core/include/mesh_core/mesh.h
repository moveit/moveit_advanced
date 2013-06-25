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

#ifndef MESH_CORE__MESH
#define MESH_CORE__MESH

#include <eigen_stl_containers/eigen_stl_containers.h>

namespace mesh_core
{

class Mesh
{
public:
  struct Triangle
  {
    int verts_[3];
    int adjacent_tris_[3];
  };

  // construct an empty mesh
  // vertices closer than epsilon will be combined.
  // degenerate triangles are removed as they are added.
  Mesh(double epsilon=0.001);

  /// remove everything and be left with empty mesh
  void clear();

  /// add a triangle defined by 3 vertices. 
  // Each vertex is a pointer to 3 // doubles.
  void add(double *a,
           double *b,
           double *c);

  /// add a triangle defined by 3 vertices. 
  void add(const Eigen::Vector3d& a,
           const Eigen::Vector3d& b,
           const Eigen::Vector3d& c);

  /// add a list of triangles.
  // tris is a pointer to ntris*3 ints.  Each triangle is represented by 3
  // adjacent ints.  The ints are indices into the verts array.
  void add(const EigenSTL::vector_Vector3d& verts,
           int ntris,
           int *tris);

  /// add a list of triangles.
  // tris is a pointer to ntris*3 ints.  Each triangle is represented by 3
  // adjacent ints.  The ints are vertex indices.
  // verts is an array of 
  void add(double *verts,
           int ntris,
           int *tris);
  

  // access. Low level - no check on idx!
  const Eigen::Vector3d& getVert(int idx) const { return verts_[idx]; }
  const Triangle& getTri(int idx) const { return tris_[idx]; }
  int getTriCount() const { return tris_.size(); }
  int getVertCount() const { return verts_.size(); }

  const EigenSTL::vector_Vector3d& getVerts() const { return verts_; }
  const std::vector<Triangle>& getTris() const { return tris_; }

private:
  double epsilon_;
  EigenSTL::vector_Vector3d verts_;
  std::vector<Triangle> tris_;
};

}


#endif
