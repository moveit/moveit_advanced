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

#include <mesh_core/mesh.h>
#include <console_bridge/console.h>

mesh_core::Mesh::Mesh(double epsilon)
  : have_degenerate_edges_(false)
{
  setEpsilon(epsilon);
}

void mesh_core::Mesh::clear()
{
  verts_.clear();
  tris_.clear();
}

void mesh_core::Mesh::reserve(int ntris, int nverts)
{
  tris_.reserve(tris_.size() + ntris);
  if (nverts == 0)
    nverts = ntris * 3;
  verts_.reserve(verts_.size() + nverts);
}

void mesh_core::Mesh::add(
      const Eigen::Vector3d& a,
      const Eigen::Vector3d& b,
      const Eigen::Vector3d& c)
{
  verts_.reserve(verts_.size()+3);
  int ai = addVertex(a);
  int bi = addVertex(b);
  int ci = addVertex(c);
  if (ai == bi || ai == ci || bi == ci)
    return;

  int tri_idx = tris_.size();
  tris_.resize(tri_idx + 1);
  Triangle &t = tris_.back();
  t.verts_[0] = ai;
  t.verts_[1] = bi;
  t.verts_[2] = ci;

  // find 3 edges
  for (int i = 0; i < 3 ; ++i)
  {
    t.edges_[i] = addEdge(t.verts_[i], t.verts_[(i+1)%3]);
    Edge& edge = edges_[t.edges_[i]];

    if (edge.tris_[0] == -2)
    {
      // >3 triangles share this edge
      edge.tris_[1]++;
    }
    else
    {
      for (int j = 0 ;; ++j)
      {
        if (j == 2)
        {
          // 3 triangles share this edge
          edge.tris_[0] = -2;
          edge.tris_[1] = 3;
          have_degenerate_edges_ = true;
          break;
        }
        if (edge.tris_[j] < 0)
        {
          edge.tris_[j] = tri_idx;
          break;
        }
      }
    }
  }
}

void mesh_core::Mesh::add(
      double *a,
      double *b,
      double *c)
{
  Eigen::Vector3d av(a[0], a[1], a[2]);
  Eigen::Vector3d bv(b[0], b[1], b[2]);
  Eigen::Vector3d cv(a[0], c[1], c[2]);
  add(av, bv, cv);
}

void mesh_core::Mesh::add(
      const EigenSTL::vector_Vector3d& verts,
      int ntris,
      int *tris)
{
  tris_.reserve(tris_.size() + ntris);
  verts_.reserve(verts_.size() + (ntris * 3));
  for (int i = 0; i < ntris ; ++i)
  {
    add(verts[tris[i*3+0]],
        verts[tris[i*3+1]],
        verts[tris[i*3+2]]);
  }
}

void mesh_core::Mesh::add(
      double *verts,
      int ntris,
      int *tris)
{
  tris_.reserve(tris_.size() + ntris);
  verts_.reserve(verts_.size() + (ntris * 3));
  for (int i = 0; i < ntris ; ++i)
  {
    add(&verts[tris[i*3+0] * 3],
        &verts[tris[i*3+1] * 3],
        &verts[tris[i*3+2] * 3]);
  }
}


void mesh_core::Mesh::setEpsilon(double epsilon)
{
  epsilon_ = std::abs(epsilon);
  epsilon_squared_ = epsilon_ * epsilon_;
  if (epsilon_squared_ < std::numeric_limits<double>::epsilon())
    epsilon_squared_ = std::numeric_limits<double>::epsilon();
}

bool mesh_core::Mesh::Triangle::operator==(const Triangle& b) const
{
  for (int i = 0 ; i < 3 ; i++)
  {
    if (verts_[0] != b.verts_[i])
      continue;
    if (verts_[1] != b.verts_[(i+1)%3])
      continue;
    if (verts_[2] != b.verts_[(i+2)%3])
      continue;
    return true;
  }
  return false;
}

int mesh_core::Mesh::addVertex(const Eigen::Vector3d& a)
{
  EigenSTL::vector_Vector3d::const_iterator it = verts_.begin();
  EigenSTL::vector_Vector3d::const_iterator end = verts_.end();
  for ( ; it != end ; ++it)
  {
    if ((*it - a).squaredNorm() < epsilon_squared_)
      return it - verts_.begin();
  }
  verts_.push_back(a);
  return verts_.size() - 1;
}

int mesh_core::Mesh::addEdge(int vertidx_a, int vertidx_b)
{
  assert(vertidx_a != vertidx_b);

  if (vertidx_a > vertidx_b)
    std::swap(vertidx_a, vertidx_b);

  std::pair<int,int> key(vertidx_a, vertidx_b);

  std::map<std::pair<int,int>,int>::const_iterator it = edge_map_.find(key);
  if (it != edge_map_.end())
    return it->second;

  int edge_idx = edges_.size();
  edges_.resize(edge_idx + 1);
  Edge &e = edges_.back();
  e.verts_[0] = vertidx_a;
  e.verts_[1] = vertidx_b;
  e.tris_[0] = -1;
  e.tris_[1] = -1;

  edge_map_[key] = edge_idx;
  return edge_idx;
}

