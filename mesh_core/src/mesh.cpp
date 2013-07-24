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
#include <mesh_core/geom.h>
#include <mesh_core/aabb.h>
#include <mesh_core/bounding_sphere.h>
#include <console_bridge/console.h>
#include <set>
#include <boost/math/constants/constants.hpp>

bool mesh_core::Mesh::debug_ = false;
bool mesh_core::Mesh::debug_this_node_ = false;
bool mesh_core::Mesh::debug_this_gap_ = false;
int mesh_core::Mesh::debug_node_id_ = -1;
int mesh_core::Mesh::debug_gap_id_ = -1;

bool acorn_debug_show_vertex_consolidate = false;
bool acorn_debug_ear_state = false;

int acorn_debug_current_node = -1;
int acorn_debug_current_gap = -1;

mesh_core::Mesh::Mesh(double epsilon)
  : have_degenerate_edges_(false)
  , number_of_submeshes_(-1)
  , adjacent_tris_valid_(false)
  , aabb_valid_(false)
  , windings_fixed_(false)
  , bounding_sphere_valid_(false)
{
  setEpsilon(epsilon);
}

void mesh_core::Mesh::clear()
{
  verts_.clear();
  tris_.clear();
  edges_.clear();
  edge_map_.clear();
  vert_info_.clear();
  have_degenerate_edges_ = false;
  number_of_submeshes_ = -1;
  submesh_tris_.clear();
  adjacent_tris_valid_ = false;
  windings_fixed_ = false;
}

void mesh_core::Mesh::reserve(int ntris, int nverts)
{
  tris_.reserve(tris_.size() + ntris);
  if (nverts == 0)
    nverts = ntris * 3;
  verts_.reserve(verts_.size() + nverts);
  edges_.reserve(nverts);
  vert_info_.reserve(nverts);
}

void mesh_core::Mesh::getTris(std::vector<int>& tris) const
{
  int ntris = tris_.size();
  tris.clear();
  tris.resize(ntris * 3);
  for (int i = 0 ; i < ntris ; ++i)
  {
    tris[i*3 + 0] = tris_[i].verts_[0];
    tris[i*3 + 1] = tris_[i].verts_[1];
    tris[i*3 + 2] = tris_[i].verts_[2];
  }
}

void mesh_core::Mesh::add(
      int a,
      int b,
      int c)
{
  if (a == b || a == c || b == c)
    return;

  int tri_idx = tris_.size();
  tris_.resize(tri_idx + 1);
  Triangle &t = tris_.back();
  t.verts_[0] = a;
  t.verts_[1] = b;
  t.verts_[2] = c;
  t.submesh_ = -1;
  t.mark_ = 0;

  number_of_submeshes_ = -1; // need to recalculate
  adjacent_tris_valid_ = false;
  windings_fixed_ = false;

  if (acorn_debug_show_vertex_consolidate)
  {
    logInform("Add tri[%d] with vidx=%d %d %d",
      tri_idx, a,b,c);
  }

  // find 3 edges
  for (int dir = 0; dir < 3 ; ++dir)
  {
    TriEdge& te = t.edges_[dir];
    te.adjacent_tri_ = -1;
    te.adjacent_tri_back_dir_ = -1;
    te.edge_idx_ = addEdge(t.verts_[dir], t.verts_[(dir+1)%3]);

    Edge& edge = edges_[te.edge_idx_];
    te.edge_tri_idx_ = edge.tris_.size();
    edge.tris_.resize(te.edge_tri_idx_ + 1);

    EdgeTri& et = edge.tris_[te.edge_tri_idx_];
    et.tri_idx_ = tri_idx;
    et.tri_dir_ = dir;

    // have more than 2 tris per edge?
    if (edge.tris_.size() > 2)
      have_degenerate_edges_ = true;

    // add to per vertex list of tris
    vert_info_[t.verts_[dir]].tris_.push_back(tri_idx);
  }

  assertValidTri_PreAdjacentValid(t, "add tri");
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
  add(ai, bi, ci);
}


void mesh_core::Mesh::add(
      double *a,
      double *b,
      double *c)
{
  Eigen::Vector3d av(a[0], a[1], a[2]);
  Eigen::Vector3d bv(b[0], b[1], b[2]);
  Eigen::Vector3d cv(c[0], c[1], c[2]);
  add(av, bv, cv);
}

void mesh_core::Mesh::add(
      int ntris,
      int *tris,
      const EigenSTL::vector_Vector3d& verts)
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
      int ntris,
      int *tris,
      double *verts)
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


void mesh_core::Mesh::findSubmeshes()
{
  setAdjacentTriangles();

  if (number_of_submeshes_ > 0)
    return;

  submesh_tris_.clear();

  std::vector<Triangle>::iterator it = tris_.begin();
  std::vector<Triangle>::iterator end = tris_.end();
  for ( ; it != end ; ++it)
  {
    it->submesh_ = -1;
  }

  number_of_submeshes_ = 0;

  for (it = tris_.begin() ; it != end ; ++it)
  {
    if (it->submesh_ == -1)
    {
      submesh_tris_.resize(number_of_submeshes_+1);
      submesh_tris_[number_of_submeshes_] = it - tris_.begin();

      assignSubmesh(*it, number_of_submeshes_);
      number_of_submeshes_++;
    }
  }
}

void mesh_core::Mesh::assignSubmesh(Triangle& t, int submesh)
{
  t.submesh_ = submesh;
  for (int dir = 0 ; dir < 3 ; ++dir)
  {
    int tadj_idx = t.edges_[dir].adjacent_tri_;
    if (tadj_idx < 0)
      continue;
    Triangle& tadj = tris_[tadj_idx];

    if (tadj.submesh_ != submesh)
      assignSubmesh(tadj, submesh);
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
    {
      if (acorn_debug_show_vertex_consolidate)
      {
        logInform("Use existing vertex (%8.4f %8.4f %8.4f) idx=%d     dist=%f  eps=%f  dsq=%f  esq=%f",
          it->x(),
          it->y(),
          it->z(),
          int(it - verts_.begin()),
          (*it - a).norm(),
          epsilon_,
          (*it - a).squaredNorm(),
          epsilon_squared_);
        logInform("                for (%8.4f %8.4f %8.4f)",
          a.x(),
          a.y(),
          a.z());
      }
      return it - verts_.begin();
    }
  }
  verts_.push_back(a);

  vert_info_.resize(verts_.size());
  vert_info_[verts_.size() - 1].edges_.reserve(7);
  vert_info_[verts_.size() - 1].tris_.reserve(7);

  if (acorn_debug_show_vertex_consolidate)
  {
    logInform("Use NEW      vertex (%8.4f %8.4f %8.4f) idx=%d   dist=%f  eps=%f  dsq=%f  esq=%f",
      verts_[verts_.size()-1].x(),
      verts_[verts_.size()-1].y(),
      verts_[verts_.size()-1].z(),
      int(verts_.size() - 1),
      (*it - a).norm(),
      epsilon_,
      (*it - a).squaredNorm(),
      epsilon_squared_);
    logInform("                for (%8.4f %8.4f %8.4f)",
      a.x(),
      a.y(),
      a.z());
  }

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
  e.tris_.reserve(2);
  e.hack_ignore_gap_ = false;

  edge_map_[key] = edge_idx;

  // add edge to per-vertex list of edges
  for (int i = 0 ; i < 2 ; ++i)
  {
    vert_info_[e.verts_[i]].edges_.push_back(edge_idx);
  }

  return edge_idx;
}


void mesh_core::Mesh::print() const
{
  logInform("Mesh 0x%08lx",(long)this);
  for (int i = 0; i < verts_.size() ; ++i)
  {
    std::stringstream ss_tris;
    std::stringstream ss_edges;
    for (int j = 0 ; j < vert_info_[i].tris_.size() ; ++j)
    {
      ss_tris << vert_info_[i].tris_[j] << ", ";
    }
    for (int j = 0 ; j < vert_info_[i].edges_.size() ; ++j)
    {
      ss_edges << vert_info_[i].edges_[j] << ", ";
    }
    logInform("   Vert[%4d] (%8.4f, %8.4f, %8.4f) tris=<%s> edges=<%s>",
      i,
      verts_[i].x(),
      verts_[i].y(),
      verts_[i].z(),
      ss_tris.str().c_str(),
      ss_edges.str().c_str());
  }
  for (int i = 0; i < tris_.size() ; ++i)
  {
    const Triangle& tri = tris_[i];
    logInform("   Tri[%4d] v(%4d,%4d,%4d) e(%4d,%4d,%4d) tadj=(%4d,%4d,%4d) sumbesh=%d",
      i,
      tri.verts_[0],
      tri.verts_[1],
      tri.verts_[2],
      tri.edges_[0].edge_idx_,
      tri.edges_[1].edge_idx_,
      tri.edges_[2].edge_idx_,
      tri.edges_[0].adjacent_tri_,
      tri.edges_[1].adjacent_tri_,
      tri.edges_[2].adjacent_tri_,
      tri.submesh_);
  }
  for (int i = 0; i < edges_.size() ; ++i)
  {
    const Edge& edge = edges_[i];
    std::stringstream ss1;
    for (int j = 0 ; j < edge.tris_.size() ; ++j)
    {
      ss1 << edge.tris_[j].tri_idx_ << ", ";
    }
    logInform("   Edge[%4d] v(%4d,%4d)  ntris=%d <%s>",
      i,
      edge.verts_[0],
      edge.verts_[1],
      int(edge.tris_.size()),
      ss1.str().c_str());
  }
}

void mesh_core::Mesh::clearMark(int value)
{
  std::vector<Triangle>::iterator it = tris_.begin();
  std::vector<Triangle>::iterator end = tris_.end();
  for ( ; it != end ; ++it)
  {
    assertValidTri(*it, "clearMark");
    it->mark_ = value;
  }
}

void mesh_core::Mesh::setAdjacentTriangles()
{
  if (adjacent_tris_valid_)
    return;

  int tri_cnt = tris_.size();
  for (int tri_idx = 0 ; tri_idx != tri_cnt ; ++tri_idx)
  {
    Triangle& tri = tris_[tri_idx];
    for (int dir = 0 ; dir < 3 ; ++dir)
    {
      TriEdge& te = tri.edges_[dir];
      te.adjacent_tri_ = -1;
      te.adjacent_tri_back_dir_ = -1;

      int edge_idx = te.edge_idx_;
      Edge& edge = edges_[edge_idx];

      ACORN_ASSERT(edge.tris_.size() > 0);
      if (edge.tris_.size() == 2)
      {
        if (edge.tris_[0].tri_idx_ == tri_idx)
        {
          te.adjacent_tri_ = edge.tris_[1].tri_idx_;
        }
        else
        {
          ACORN_ASSERT(edge.tris_[1].tri_idx_ == tri_idx);
          te.adjacent_tri_ = edge.tris_[0].tri_idx_;
        }

        Triangle& adj = tris_[te.adjacent_tri_];

        for (int back_dir = 0 ;; ++back_dir)
        {
          ACORN_ASSERT(back_dir < 3);
          if (back_dir >= 3)
            break;

          TriEdge& adj_te = adj.edges_[back_dir];
          if (adj_te.edge_idx_ == edge_idx)
          {
            te.adjacent_tri_back_dir_ = back_dir;
            break;
          }
        }

      }
      else if (edge.tris_.size() > 2)
      {
        te.adjacent_tri_ = -2;
      }
    }
  }
  adjacent_tris_valid_ = true;
}

// make all windings CCW
void mesh_core::Mesh::fixWindings()
{
  if (windings_fixed_)
    return;

  int mark = 0;

  findSubmeshes();
  clearMark(mark);


  for (int i=0; i<number_of_submeshes_; ++i)
  {
    int tri_cnt = 0;
    int flip_cnt = 0;
    fixSubmeshWindings(
              tris_[submesh_tris_[i]],
              false,
              ++mark,
              tri_cnt,
              flip_cnt);
    if (!isWindingCCW(i))
    {
      // guessed wrong - flip them all back the opposite way
      fixSubmeshWindings(
              tris_[submesh_tris_[i]],
              true,
              ++mark,
              tri_cnt,
              flip_cnt);
    }
  }

  windings_fixed_ = true;
  face_normals_.clear();

  if (1) // DEBUG
  {
    clearMark(mark);  // this sanity checks all triangles
  }
}

bool mesh_core::Mesh::isWindingCCW(int submesh) const
{
  double minx_x = std::numeric_limits<double>::max();
  int minx_vidx = -1;

  std::vector<Triangle>::const_iterator tri = tris_.begin();
  std::vector<Triangle>::const_iterator tri_end = tris_.end();
  for ( ; tri != tri_end ; ++tri)
  {
    if (tri->submesh_ != submesh)
      continue;

    for (int j = 0; j < 3 ; ++j)
    {
      if (verts_[tri->verts_[j]].x() < minx_x)
      {
        minx_x = verts_[tri->verts_[j]].x();
        minx_vidx = tri->verts_[j];
      }
    }
  }

  // no triangles in this submesh?
  if (minx_vidx < 0)
    return true;

  ACORN_ASSERT(minx_vidx < vert_info_.size());
  const Vertex& vinfo = vert_info_[minx_vidx];

  const Eigen::Vector3d& minx_vert = verts_[minx_vidx];

  // find edge with smallest abs(x/yz) slope
  double best_slope = std::numeric_limits<double>::max();
  int best_eidx = -1;
  for (int e = vinfo.edges_.size() - 1 ; e >= 0 ; --e)
  {
    int eidx = vinfo.edges_[e];
    const Edge& edge = edges_[eidx];

    // edge is part of same submesh?
    for (int t = edge.tris_.size() - 1 ; t >= 0 ; --t)
    {
      if (tris_[edge.tris_[t].tri_idx_].submesh_ == submesh)
      {
        int other_vidx = otherVertIndex(edge, minx_vidx);
        const Eigen::Vector3d& other_vert = verts_[other_vidx];

        const Eigen::Vector3d& edge_vec = other_vert - minx_vert;
        double yz = std::sqrt(edge_vec.y() * edge_vec.y() + edge_vec.z() * edge_vec.z());
        if (yz < std::numeric_limits<double>::epsilon())
          continue;
        double slope = edge_vec.x() / yz;

if (slope < 0.0)
{
  print();
  logInform("isWindingCCW negative slope %f",slope);
  logInform("minx_vidx=%4d  (%f %f %f)",minx_vidx, minx_vert.x(), minx_vert.y(), minx_vert.z());
  logInform("other_vid=%4d  (%f %f %f)",other_vidx, other_vert.x(), other_vert.y(), other_vert.z());
  logInform("edge_vex =      (%f %f %f)",edge_vec.x(), edge_vec.y(), edge_vec.z());
  logInform("minx_x=%f",minx_x);
}

        ACORN_ASSERT(slope >= 0.0);
        if (slope < best_slope)
        {
          best_slope = slope;
          best_eidx = eidx;
        }
        break;
      }
    }
  }

  ACORN_ASSERT(best_eidx >= 0);
  if (best_eidx < 0)
    return true;
  
  // of triangles touching that edge find one whose normal is closest to x axis
  double best_xabs = -1.0;
  double best_x = 0.0;
  for (int t = edges_[best_eidx].tris_.size() - 1 ; t >= 0 ; --t)
  {
    const Triangle &tri = tris_[edges_[best_eidx].tris_[t].tri_idx_];

    Eigen::Vector3d ab = verts_[tri.verts_[1]] - verts_[tri.verts_[0]];
    Eigen::Vector3d bc = verts_[tri.verts_[2]] - verts_[tri.verts_[1]];
    Eigen::Vector3d norm = ab.cross(bc).normalized();
    double xabs = std::abs(norm.x());
    if (xabs > best_xabs)
    {
      best_xabs = xabs;
      best_x = norm.x();
    }
  }

  return best_x <= 0.0;
}

bool mesh_core::Mesh::isWindingSame(Triangle& tri, int dir)
{
  int adj_idx = tri.edges_[dir].adjacent_tri_;
  if (adj_idx < 0)
    return false;
  Triangle& adj = tris_[adj_idx];

  int back_dir = tri.edges_[dir].adjacent_tri_back_dir_;
  return tri.verts_[dir] != adj.verts_[back_dir];
}

namespace {
struct FlipInfo
{
  mesh_core::Mesh::Triangle* tri;
  bool flip;
};
}

// if possible, make all windings point the same way for all triangles in this
// tris submesh.
// Should only be called from fixWindings().
// Returns the total number of triangles in the submesh and the number of triangles flipped.
// If flip_this_tri is true, the initial triangle's winding is flipped.
void mesh_core::Mesh::fixSubmeshWindings(
      Triangle& first_tri,
      bool flip_first_tri,
      int mark,
      int& tri_cnt,
      int& flip_cnt)
{
  assertValidTri(first_tri, "fixWindings1");

  std::vector<FlipInfo> list;
  list.resize(tris_.size());

  list[0].tri = &first_tri;
  list[0].flip = flip_first_tri;
  first_tri.mark_ = mark;

  FlipInfo *tail = &list[0];
  FlipInfo *head = &list[1];
  
  // go through all tris in the same submesh and make them consistent with the
  // first_tri
  do
  {
    FlipInfo &info = *tail++;
    Triangle& tri = *info.tri;
    tri_cnt++;

    assertValidTri(tri, "fixWindings2");

    if (info.flip)
    {
      flipWinding(*info.tri);
      flip_cnt++;
    }
    
    for (int dir = 0 ; dir < 3 ; ++dir)
    {
      int adj_idx = tri.edges_[dir].adjacent_tri_;
      if (adj_idx < 0)
        continue;
      Triangle& adj = tris_[adj_idx];

      assertValidTri(adj, "fixWindings3");

      // visit each triangle only once
      if (adj.mark_ == mark)
        continue;
      adj.mark_ = mark;

      head->tri = &adj;
      head->flip = !isWindingSame(tri, dir);

      head++;

      if (head - &list[0] > list.size())
      {
        logError("fixSubmeshWindings OVERFLOWED!");
        return;
      }
    }
  }
  while (tail != head);
}

// flip the winding order by swapping vertex 1 and 2
void mesh_core::Mesh::flipWinding(Triangle& tri)
{
  ACORN_ASSERT_TRI(tri);
  assertValidTri(tri, "flipWinding1");

  std::swap(tri.verts_[1], tri.verts_[2]);
  std::swap(tri.edges_[0], tri.edges_[2]);

  // fixup edge 0 and 2 which got swapped
  for (int dir = 0 ; dir < 3 ; dir++)
  {
    if (dir == 1)
      continue;

    TriEdge& te = tri.edges_[dir];
    Edge& edge = edges_[te.edge_idx_];
    EdgeTri& et = edge.tris_[te.edge_tri_idx_];
    et.tri_dir_ = dir;

    if (adjacent_tris_valid_ && te.adjacent_tri_ >= 0)
    {
      Triangle& adj = tris_[te.adjacent_tri_];
      TriEdge& adj_te = adj.edges_[te.adjacent_tri_back_dir_];
      adj_te.adjacent_tri_back_dir_ = dir;
    }
  }

  assertValidTri(tri, "flipWinding2");
}

void mesh_core::Mesh::assertValidTri(
      const Triangle& tri,
      const char *msg) const
{
  ACORN_ASSERT(adjacent_tris_valid_);
  ACORN_ASSERT(number_of_submeshes_ > 0);
  assertValidTri_PreAdjacentValid(tri, msg);
}

void mesh_core::Mesh::assertValidTri_PreAdjacentValid(
      const Triangle& tri,
      const char *msg) const
{
  ACORN_ASSERT_TRI(tri);
  ACORN_ASSERT_TRI_IDX(triIndex(tri));
  ACORN_ASSERT(vert_info_.size() == verts_.size());

  if (number_of_submeshes_ > 0)
  {
    ACORN_ASSERT(number_of_submeshes_ == submesh_tris_.size());
    ACORN_ASSERT(tri.submesh_ >= 0 && tri.submesh_ < number_of_submeshes_);
  }

  for (int dir = 0 ; dir < 3 ; ++dir)
  {
    ACORN_ASSERT_VERT_IDX(tri.verts_[dir]);
    ACORN_ASSERT_EDGE_IDX(tri.edges_[dir].edge_idx_);
    const TriEdge& te = tri.edges_[dir];
    const Edge& edge = edges_[te.edge_idx_];
    ACORN_ASSERT(te.edge_tri_idx_ <= edge.tris_.size());
    const EdgeTri& et = edge.tris_[te.edge_tri_idx_];
    assertValidEdge(edge, msg);

    ACORN_ASSERT(et.tri_idx_ == triIndex(tri));
    ACORN_ASSERT(et.tri_dir_ == dir);

    if (adjacent_tris_valid_ && te.adjacent_tri_ >= 0)
    {
      ACORN_ASSERT_TRI_IDX(te.adjacent_tri_);
      ACORN_ASSERT_DIR(te.adjacent_tri_back_dir_);
      const Triangle& adj = tris_[te.adjacent_tri_];
      int back_dir = te.adjacent_tri_back_dir_;
      const TriEdge& adj_te = adj.edges_[back_dir];

      ACORN_ASSERT(adj_te.edge_idx_ == te.edge_idx_);
      ACORN_ASSERT(adj_te.adjacent_tri_ == triIndex(tri));
      ACORN_ASSERT(adj_te.adjacent_tri_back_dir_ == dir);
    }

    ACORN_ASSERT(edge.verts_[0] < edge.verts_[1]);
    if (edge.verts_[0] == tri.verts_[dir])
    {
      ACORN_ASSERT(edge.verts_[1] == tri.verts_[(dir+1)%3]);
    }
    else
    {
      ACORN_ASSERT(edge.verts_[0] == tri.verts_[(dir+1)%3]);
      ACORN_ASSERT(edge.verts_[1] == tri.verts_[dir]);
    }
  }
}

void mesh_core::Mesh::assertValidEdge(
      const Edge& edge,
      const char *msg) const
{
  int edge_idx = &edge - &edges_[0];
  ACORN_ASSERT_EDGE_IDX(edge_idx);
  for (int i = 0 ; i < 2 ; ++i)
  {
    ACORN_ASSERT_VERT_IDX(edge.verts_[i]);
    int vidx = edge.verts_[i];
    const Vertex& vtx = vert_info_[vidx];

    int ve_cnt = 0;
    for (int j = 0 ; j < vtx.edges_.size() ; ++j)
    {
      ACORN_ASSERT_EDGE_IDX(vtx.edges_[j]);
      if (vtx.edges_[j] == edge_idx)
        ve_cnt++;
    }
    ACORN_ASSERT(ve_cnt == 1);
  }
}

#define ACORN_DEBUG_FILL_GAP 0


void mesh_core::Mesh::fillGaps()
{
  findSubmeshes();

  if (ACORN_DEBUG_FILL_GAP)
  {
    logInform("=========== Filling Gaps in mesh %08lx",long(this));
    print();
  }

  int ntris = tris_.size();

  for (;;)
  {
    const Edge* edge = findGap();
    if (!edge)
      break;
    fillGap(*edge);

    if (ntris == tris_.size())
    {
      if (!acorn_debug_ear_state)
      {
        // re-run the gap fill with debugging enabled
        acorn_debug_ear_state = true;
        continue;
      }
      logInform("Attempt to fill gap failed.  Aborting");

      print();
      ACORN_ASSERT(0); // should not happen -- abort for easier debugging

      break;
    }
    else
    {
      ntris = tris_.size();
    }
  }

  fixWindings();

  //print();
}

const mesh_core::Mesh::Edge* mesh_core::Mesh::findGap() const
{
  std::vector<Edge>::const_iterator edge = edges_.begin();
  std::vector<Edge>::const_iterator edge_end = edges_.end();
  for (; edge != edge_end ; ++edge)
  {
    if (edge->hack_ignore_gap_)
      continue;

    // edge has only 1 tri?
    if (edge->tris_.size() == 1)
    {
      if (ACORN_DEBUG_FILL_GAP)
        logInform("Found gap: edge %d has 1 tri",int(&*edge - &edges_[0]));
      return &*edge;
    }
  }

  edge = edges_.begin();
  for (; edge != edge_end ; ++edge)
  {
    if (edge->hack_ignore_gap_)
      continue;

    // edge has odd number of triangles
    if (edge->tris_.size() & 1)
    {
      if (ACORN_DEBUG_FILL_GAP)
        logInform("Found gap: edge %d has %d tri",int(&*edge - &edges_[0]), edge->tris_.size());
      return &*edge;
    }
  }

  return NULL; // no gaps found
}

struct GapEdge
{
  int vert_idx_;
  int edge_idx_;
  int tri_idx_;
  bool correct_winding_;
  bool connected_to_same_tri_;
};

void mesh_core::Mesh::fillGap(const Edge& first_edge)
{
  bool save_acorn_debug_ear_state = acorn_debug_ear_state;
  debug_this_gap_ = (debug_ && debug_this_node_ && debug_gap_id_ != -1 && debug_gap_id_ == gap_debug_.size());
  acorn_debug_ear_state = debug_this_gap_;
  

  ACORN_ASSERT(first_edge.tris_.size() == 1);

  if (0 && ACORN_DEBUG_FILL_GAP)
    print();
  if (ACORN_DEBUG_FILL_GAP)
  {
    logInform("================ Fill gap in edge[%d]  v(%d,%d)",
      edgeIndex(first_edge),
      first_edge.verts_[0],
      first_edge.verts_[1]);
  }

  const EdgeTri& first_et = first_edge.tris_[0];
  Triangle &first_tri = tris_[first_et.tri_idx_];

  std::vector<GapEdge> loop;
  loop.reserve(tris_.size());
  int current_vtx = -1;
  int current_edge = edgeIndex(first_edge);

  loop.resize(1);
  loop[0].edge_idx_ = edgeIndex(first_edge);
  loop[0].correct_winding_ = true;
  loop[0].tri_idx_ = first_et.tri_idx_;

  // follow edge in triangle's increasing-dir order 
  if (first_tri.verts_[first_et.tri_dir_] == first_edge.verts_[0])
  {
    loop[0].vert_idx_ = first_edge.verts_[0];
    current_vtx = first_edge.verts_[1];
  }
  else
  {
    ACORN_ASSERT(first_tri.verts_[first_et.tri_dir_] == first_edge.verts_[1]);
    loop[0].vert_idx_ = first_edge.verts_[1];
    current_vtx = first_edge.verts_[0];
  }

  std::vector<GapEdge> new_edges;
  new_edges.reserve(10);

  // look for more gap edges until we have a loop
  int loop_start_index = -1;
  do
  {
    Vertex& vtx = vert_info_[current_vtx];
    new_edges.clear();
    bool found_correct_winding = false;

    if (ACORN_DEBUG_FILL_GAP)
      logInform("  check edges for vert[%d]",current_vtx);

    // check all edges touching current vertex
    for (int i = 0 ; i < vtx.edges_.size() ; ++i)
    {
      if (ACORN_DEBUG_FILL_GAP)
        logInform("   check edge[%d] v(%d,%d)  ntris=%d",
          vtx.edges_[i],
          edges_[vtx.edges_[i]].verts_[0],
          edges_[vtx.edges_[i]].verts_[1],
          edges_[vtx.edges_[i]].tris_.size());

      if (vtx.edges_[i] == current_edge)
        continue;

      Edge& vedge = edges_[vtx.edges_[i]];
      if ((vedge.tris_.size() & 1) == 0)
        continue;

      GapEdge ge;
      ge.vert_idx_ = current_vtx;
      ge.edge_idx_ = vtx.edges_[i];
      ge.tri_idx_ = -1; // filled in below
      ge.connected_to_same_tri_ = false;
      ge.correct_winding_ = false;

      // see if there are any triangles along this edge with the correct winding
      for (int j = 0 ; j < vedge.tris_.size() ; ++j)
      {
        EdgeTri& et = vedge.tris_[j];
        Triangle& tri = tris_[et.tri_idx_];
        ge.tri_idx_ = et.tri_idx_;

        // does the edge go in positive winding around triangle?
        if (tri.verts_[et.tri_dir_] == current_vtx)
        {
          if (!found_correct_winding)
          {
            new_edges.clear();
            found_correct_winding = true;
          }
          ge.correct_winding_ = true;
          new_edges.push_back(ge);
        }
        else if (!found_correct_winding)
        {
          new_edges.push_back(ge);
        }
      }
    }

    // TODO: remove this workaround
    if (new_edges.empty())
    {
      logWarn("Unable to close gap in edge %d -- ignoring.",
          int(&first_edge - &edges_[0]));
      edges_[edgeIndex(first_edge)].hack_ignore_gap_ = true;
      return;
    }

    ACORN_ASSERT(!new_edges.empty());

    // I expect to be able to find a winding in the correct direction for
    // most meshes.
    if (!found_correct_winding)
    {
      logWarn("mesh_core::Mesh::fillGap() found no good edges");
    }

    if (new_edges.size() > 1)
    {
      // this is not necessarily an error.  Just means complex cracks.
      logInform("Found %d gap edges at vtx %d",
        int(new_edges.size()), 
        current_vtx);


      // TODO: use some heuristic to decide which edge to follow?
    }

    ACORN_ASSERT(!new_edges.empty());
    if (new_edges.empty())
    {
      loop_start_index = 0;
      break;
    }

    // Now we have a number of candidate edges.  Use the first one
    loop.push_back(new_edges[0]);

    GapEdge& ge = loop.back();
    Edge& edge = edges_[ge.edge_idx_];

    current_edge = ge.edge_idx_;
    current_vtx = otherVertIndex(edge, current_vtx);

    // check to see if we have a loop.
    for (int i = loop.size() - 2 ; i >= 0 ; --i)
    {
      if (loop[i].vert_idx_ == current_vtx)
      {
        loop_start_index = i;
        break;
      }
    }
  }
  while (loop_start_index == -1);

#if 1||ENABLE_DEBUGGING
  GapDebugInfo* db = NULL;
  if (debug_)
  {
    gap_debug_.resize(gap_debug_.size() + 1);
    db = &gap_debug_.back();
    int nloop = loop.size() - loop_start_index;
    db->points_.resize(nloop);
    db->verts_.resize(nloop);
    db->neigbor_tris_.resize(nloop);
    db->gap_tris_.reserve(nloop);
    db->pre_gap_tri_cnt_ = tris_.size();

    for (int i = 0 ; i < nloop ; ++i)
    {
      GapEdge& ge = loop[loop_start_index + i];
      db->verts_[i] = ge.vert_idx_;
      db->points_[i] = verts_[ge.vert_idx_];
      db->neigbor_tris_[i] = ge.tri_idx_;
    }
  }
#endif

  // Now we have a loop of vertices around the gap.
  // Generate triangles to fill in the gap

  std::vector<int> loop_verts;
  loop_verts.reserve(loop.size() - loop_start_index);
  for (int i = loop_start_index ;  i < loop.size() ; ++i)
  {
    loop_verts.push_back(loop[i].vert_idx_);
  }

  ACORN_ASSERT(loop_verts.size() >= 3);

  generatePolygon(loop_verts, true);

  findSubmeshes();

#if 1||ENABLE_DEBUGGING
  if (db)
  {
    for (int i = db->pre_gap_tri_cnt_ ; i < tris_.size() ; i++)
      db->gap_tris_.push_back(i);
  }
#endif
  acorn_debug_ear_state = save_acorn_debug_ear_state;
}

static inline double cross2d(const Eigen::Vector2d& a, const Eigen::Vector2d& b)
{
  return a.x() * b.y() - a.y() * b.x();
}

struct mesh_core::Mesh::GapPoint
{
  enum State
  {
    CONVEX,
    REFLEX,
    EAR,


    SELF_INTERSECT,
    TURN_180,
    OK,
  };

  State state_;
  int orig_vert_idx_;       // index of vertex in mesh
  int idx_;                 // index in the points array.  For debugging.

  Eigen::Vector2d v2d_;     // vertex

  // these 4 elements describe the edge between this and the next point
  LineSegment2D seg_;       // for finding self intersecting edges
  Eigen::Vector2d delta_;   // next.v2d_ - v2d_
  Eigen::Vector2d norm_;    // norm - perpendicular normal pointing inside
  double d_;                // -(norm_ dot v2d_)

  GapPoint *next_;
  GapPoint *prev_;

  void calculateEdgeAttributes()
  {
    delta_ = next_->v2d_ - v2d_;
    norm_.x() = -delta_.y();
    norm_.y() =  delta_.x();
    d_ = -norm_.dot(v2d_);
    seg_.initialize(v2d_, next_->v2d_);
  }

  void erase()
  {
    next_->prev_ = prev_;
    prev_->next_ = next_;
    prev_->calculateEdgeAttributes();
  }

  void print(const char *pfx = "")
  {
    logInform("%spt[%4d] v2d=(%8.4f %8.4f) delta=(%8.4f %8.4f) norm=(%8.4f %8.4f) d=%8.4f  cross=%10.6f",
      pfx,
      idx_,
      v2d_.x(),
      v2d_.y(),
      delta_.x(),
      delta_.y(),
      norm_.x(),
      norm_.y(),
      d_,
      cross2d(prev_->delta_, delta_));
    ACORN_ASSERT(std::abs(norm_.dot(v2d_) + d_) <= std::numeric_limits<double>::epsilon());
    ACORN_ASSERT(std::abs(norm_.dot(next_->v2d_) + d_) <= std::numeric_limits<double>::epsilon());
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Decide if point is an ear or not.
void mesh_core::Mesh::calcEarState(GapPoint* point)
{
  if (acorn_debug_ear_state)
  {
    logInform("Calc EarState for gap points[%d]",point->idx_);
  }

  double cr = cross2d(point->prev_->delta_, point->delta_);
  if (acorn_debug_ear_state)
  {
    logInform("  cr for points[%d] = %f", point->idx_, cr);
  }
  if (cr < 0.0)
  {
    point->state_ = GapPoint::REFLEX;
    if (acorn_debug_ear_state)
    {
      logInform("                state=REFLEX");
    }
    return;
  }

  GapPoint* p[3];
  p[0] = point->prev_;
  p[1] = point;
  p[2] = point->next_;

  ACORN_ASSERT(p[0] != p[1]);
  ACORN_ASSERT(p[0] != p[2]);

  // generate new p[2] with an edge from p[2] to p[0]
  GapPoint p2;
  p2.idx_ = p[2]->idx_;
  p2.v2d_ = p[2]->v2d_;
  p2.next_ = p[0];
  p2.prev_ = p[1];
  p2.calculateEdgeAttributes();
  p[2] = &p2;

  if (acorn_debug_ear_state)
  {
    p[0]->print("      ");
    p[1]->print("      ");
    p[2]->print("      ");
  }

  GapPoint* pt = point->next_->next_;
  GapPoint* pt_end = point->prev_;
  for ( ; pt != pt_end ; pt = pt->next_)
  {
    if (acorn_debug_ear_state)
    {
      logInform("                   check point[%d] (%8.4f %8.4f)",
        pt->idx_,
        pt->v2d_.x(),
        pt->v2d_.y());
    }

    for (int j = 0;; ++j)
    {
      // point is inside triangle.  Not an ear.
      if (j == 3)
      {
        point->state_ = GapPoint::CONVEX;
        if (acorn_debug_ear_state)
        {
          logInform("                state=CONVEX");
        }
        return;
      }


      // outside the tri?  check next pt
      double dist = p[j]->norm_.dot(pt->v2d_) + p[j]->d_;

      if (acorn_debug_ear_state)
        logInform("                      dist[%d]=%f",j,dist);

      if (dist <= 0.0)
        break;
    }
  }

  if (acorn_debug_ear_state)
  {
    logInform("                state=EAR");
  }
  point->state_ = GapPoint::EAR;
}

void mesh_core::Mesh::addGapTri(
      int a,
      int b,
      int c)
{
  if (!acorn_debug_ear_state)
  {
    add(a,b,c);
  }
  else
  {
    int tri_idx = tris_.size();
    add(a,b,c);
    if (acorn_debug_ear_state && debug_ && !gap_debug_.empty() && tri_idx != tris_.size())
    {
      int base = 0;
      if (debug_ && !gap_debug_.empty())
        base = gap_debug_.back().pre_gap_tri_cnt_;

      logInform("  Add_Gap_Triangle tri_idx=%d gap_tri_idx=%d  v=%d %d %d",
        tri_idx,
        tri_idx - base,
        a,b,c);
    }
  }
}

void mesh_core::Mesh::addGapTri(
      const GapPoint *p,
      double direction)
{
  if (direction < 0.0)
  {
    addGapTri(
        p->prev_->orig_vert_idx_,
        p->orig_vert_idx_,
        p->next_->orig_vert_idx_);
  }
  else
  {
    addGapTri(
        p->prev_->orig_vert_idx_,
        p->next_->orig_vert_idx_,
        p->orig_vert_idx_);
  }

}

// generate a polygon given a loop of vertex indices
// if partial_ok is true then it will add at least one tri but may not close
// the entire oplygon.  Otherwise it will close the entire polygon even if the
// polygon has holes.
void mesh_core::Mesh::generatePolygon(
    const std::vector<int> verts,
    bool partial_ok)
{
  GapDebugInfo* db = NULL;
  if (debug_ && !gap_debug_.empty())
  {
    db = &gap_debug_.back();
  }

  int nverts = verts.size();

  if (acorn_debug_ear_state)
  {
    logInform("#####################");
    logInform("#####################");
    logInform("#####################");
    logInform("#####################");
    logInform("##################### generatePolygon nverts=%d",nverts);
  }

  if (nverts < 3)
    return;

  if (nverts <= 4)
  {
    addGapTri(verts[0], verts[2], verts[1]);
    if (nverts == 4)
      addGapTri(verts[0], verts[3], verts[2]);
    return;
  }

  // get array of verts
  EigenSTL::vector_Vector3d points3d;
  points3d.resize(nverts);
  for (int i = 0 ; i < nverts ; i++)
    points3d[i] = verts_[verts[i]];

  // find a plane through (or near) the points
  PlaneProjection proj(points3d);

  if (acorn_debug_ear_state)
  {
    for (int i = 0 ; i < nverts ; ++i)
    {
      logInform("  vert[%4d] = %s",i,mesh_core::str(verts_[verts[i]]).c_str());
    }
    logInform("proj=\n%s", proj.str().c_str());
  }

  // project points onto the plane
  std::vector<GapPoint> points;
  points.resize(nverts);
  for (int i = 0 ; i < nverts ; i++)
  {
    GapPoint* p = &points[i];
    GapPoint* pn = &points[(i+1)%nverts];
    p->next_ = pn;
    pn->prev_ = p;

    p->state_ = GapPoint::OK;
    p->orig_vert_idx_ = verts[i];
    p->idx_ = i;
    p->v2d_ = proj.project(verts_[verts[i]]);
  }

  GapPoint* point0 = &points[0];

  // calculate edge attributes and seg_
  GapPoint* p = point0;
  do
  {
    p->calculateEdgeAttributes();
    p = p->next_;
  }
  while (p != point0);


  // check for 180 degree turns and linear loops
  for (;;)
  {
    int linear_cnt = 0;
    GapPoint* last_180 = NULL;

    // check for 180 degree turns
    p = point0;
    do
    {
      GapPoint* pn = p->next_;
      double cr = cross2d(p->delta_, pn->delta_);
      if (std::abs(cr) <= std::numeric_limits<double>::epsilon())
      {
        if (p->delta_.dot(pn->delta_) < 0.0)
        {
          pn->state_ = GapPoint::TURN_180;
          last_180 = pn;
        }
        linear_cnt++;
      }
      p = pn;
    }
    while (p != point0);

    if (!last_180)
      break;

    logInform("Found %d linear verts from nverts=%d",linear_cnt, nverts);

    // entire loop is linear.  Close off one triangle and abort (try again).
    if (linear_cnt == nverts)
    {
      addGapTri(last_180->prev_->orig_vert_idx_,
                last_180->orig_vert_idx_,
                last_180->next_->orig_vert_idx_);

      ACORN_ASSERT(partial_ok);
      return;
    }

    logInform("Eliminate 180 degree turn in vert %d",last_180->idx_);

    // found a 180 degree turn.  Eliminate it and recheck.
    last_180->erase();
    if (point0 == last_180)
      point0 = last_180->next_;
    nverts--;
    last_180->prev_->state_ = GapPoint::OK;   // will be rechecked

    if (nverts == 4)
    {
      GapPoint* a = point0;
      GapPoint* b = a->next_;
      GapPoint* c = b->next_;
      GapPoint* d = c->next_;
      addGapTri(a->orig_vert_idx_,
                c->orig_vert_idx_,
                b->orig_vert_idx_);
      addGapTri(a->orig_vert_idx_,
                d->orig_vert_idx_,
                c->orig_vert_idx_);
      fixWindings();
      return;
    }
  }
  
  // check for self intersections
  {
    GapPoint* intersect_a = NULL;
    GapPoint* intersect_b = NULL;
    int self_intersect_cnt = 0;
    p = point0;
    do
    {
      for (GapPoint* p2 = p->next_->next_; p2 != point0; p2 = p2->next_)
      {
        Eigen::Vector2d intersection;
        bool parallel;
        if (p->seg_.intersect(p2->seg_, intersection, parallel) && p2->next_ != p)
        {
          p->state_ = GapPoint::SELF_INTERSECT;
          p2->state_ = GapPoint::SELF_INTERSECT;
          self_intersect_cnt++;
          if (!intersect_a ||
              p->next_->next_ == p2 ||
              p2->next_->next_ == p)
          {
            // save off one intersection.  Prefer intersections between nearby verts.
            intersect_a = p;
            intersect_b = p2;
          }
          if (acorn_debug_ear_state)
          {
            logInform("   INTERSECTION: V[%4d]-V[%4d] intersects with V[%4d]-V[%4d]",
              p->idx_,
              p->next_->idx_,
              p2->idx_,
              p2->next_->idx_);
          }
        }
      }
      p = p->next_;
    }
    while (p != point0);

    // if self intersecting, find a loop that does not self intersect
    if (intersect_a)
    {
      logInform("Found %d self intersections  (node=%d gap=%d)",
        self_intersect_cnt,
        acorn_debug_current_node,
        gap_debug_.size() - 1);

      GapPoint* best_start = NULL;
      GapPoint* best_end = NULL;
      GapPoint* start = NULL;
      int best_cnt = -1;
      int cnt = 0;
      
      bool done = false;
      for (p = intersect_a ; !done ; p = p->next_)
      {
        if (p->state_ == GapPoint::SELF_INTERSECT)
        {
          done = (p == intersect_a && best_cnt != -1);
          if (cnt > best_cnt)
          {
            best_cnt = cnt;
            best_start = start;
            best_end = p->prev_;
          }
          start = p->next_;
          cnt = 0;
        }
        else
        {
          cnt++;
        }
      }

      logInform("found nonintersecting loop of best_cnt=%d verts",best_cnt);

      // if everything self intersects then just connect one of the intersecting pairs
      if (best_cnt < 3)
      {
        int v0 = intersect_a->orig_vert_idx_;
        int v1 = intersect_a->next_->orig_vert_idx_;
        int v2 = intersect_b->orig_vert_idx_;
        int v3 = intersect_b->next_->orig_vert_idx_;
        addGapTri(v0, v2, v1);
        addGapTri(v0, v3, v2);
        fixWindings();
        ACORN_ASSERT(partial_ok);
        return;
      }

      // otherwise we have a new, smaller loop with no intersections.  Continue.
      point0 = best_start;
      best_start->prev_ = best_end;
      best_end->next_ = best_start;
      best_end->calculateEdgeAttributes();
      nverts = best_cnt;

      // if we have a loop with 3-4 verts, close it.
      if (nverts <= 4)
      {
        GapPoint* a = point0;
        GapPoint* b = a->next_;
        GapPoint* c = b->next_;
        GapPoint* d = c->next_;
        addGapTri(a->orig_vert_idx_,
                  c->orig_vert_idx_,
                  b->orig_vert_idx_);
        if (d != a)
          addGapTri(a->orig_vert_idx_,
                    d->orig_vert_idx_,
                    c->orig_vert_idx_);
        fixWindings();
        ACORN_ASSERT(partial_ok);
        return;
      }

    }
  }

  if (acorn_debug_ear_state)
  {
    logInform("Create polygon for %d verts:",nverts);
    p = point0;
    do
    {
      p->print("  ");
      p = p->next_;
    }
    while (p != point0);
  }
  
  // find winding direction
  // use cross product sign at extreme (xmin) point, guaranteed to be convex unless colinear.
  // point.cross_ and direction have same sign if corner is convex.
  // direction is positive if loop is ccw
  double direction = 1.0;
  {
    // find point with min x coord
    GapPoint* xmin = point0;
    for (p = point0->next_ ; p != point0 ; p = p->next_)
    {
      if (p->v2d_.x() < xmin->v2d_.x())
        xmin = p;
    }

    if (acorn_debug_ear_state)
    {
      logInform("  xmin = points[%d]",xmin->idx_);
    }

    // look for a corner which bends more than 0 degrees to see the direction of outside corners
    p = xmin;
    do
    {
      direction = cross2d(p->prev_->delta_, p->delta_);
      if (acorn_debug_ear_state)
        logInform("  direction (cross) at points[%d] = %f", p->idx_, direction);
      if (std::abs(direction) > std::numeric_limits<double>::epsilon())
        break;

      p = p->next_;
    }
    while (p != xmin);
  }

  // if direction is backwards, reverse direction of points so they are CCW around polygon
  if (direction < 0.0)
  {
    p = point0;
    do
    {
      GapPoint *pn = p->next_;
      GapPoint *pp = p->prev_;

      p->next_ = pp;
      p->prev_ = pn;
      p->calculateEdgeAttributes();

      p = pn;
    }
    while (p != point0);

    if (acorn_debug_ear_state)
    {
      logInform("Create polygon for %d verts REVERSED:",nverts);
      p = point0;
      do
      {
        p->print("  ");
        p = p->next_;
      }
      while (p != point0);
    }
  }

  // categorize each point
  p = point0;
  do
  {
    calcEarState(p);
    p = p->next_;
  }
  while (p != point0);

  ACORN_ASSERT(nverts > 4);

  int cnt = 0;
  p = point0;
  for (;;)
  {
    if (acorn_debug_ear_state)
    {
      logInform("  Check point %d  EarState=%d",p->idx_,int(p->state_));
    }

    if (p->state_ != GapPoint::EAR)
    {
      cnt++;
      if (cnt > nverts)
      {
        logWarn("  No ears and %d verts left in polygon -- ABORT",nverts);
        return;
      }

      p = p->next_;
      continue;
    }

    // Found an ear.

    cnt = 0;

    int old_tri_size = tris_.size();
    if (acorn_debug_ear_state)
    {
      logInform("  Fill Ear    %d  as tri %d", p->idx_, tris_.size());
    }

    addGapTri(p, direction);

    if (acorn_debug_ear_state && old_tri_size+1 != tris_.size())
    {
      logInform("  NO TRIANGLE CREATED!!");
    }


    p->erase();
    if (p == point0)
      point0 = p->prev_;
    --nverts;
    p = p->prev_;

    if (nverts == 4)
      break;

    calcEarState(p);
    if (p->state_ != GapPoint::EAR)
    {
      p = p->next_;
      calcEarState(p);
    }
  }

  ACORN_ASSERT(nverts == 4);

  // last 4 verts - add last 2 tris
  addGapTri(p, direction);
  addGapTri(p->next_->next_, direction);
}

int acorn_db_slice_showmin=-1;
int acorn_db_slice_showmax=-1;
int acorn_db_slice_showclip=-1;
int acorn_db_slice_current=0;
Eigen::Vector3d acorn_db_slice_in = Eigen::Vector3d(0,0,0);
Eigen::Vector3d acorn_db_slice_out = Eigen::Vector3d(0,0,0);
Eigen::Vector3d acorn_db_slice_clip = Eigen::Vector3d(0,0,0);
EigenSTL::vector_Vector3d acorn_db_slice_pts_clip;
EigenSTL::vector_Vector3d acorn_db_slice_pts_0;
EigenSTL::vector_Vector3d acorn_db_slice_pts_in;
EigenSTL::vector_Vector3d acorn_db_slice_pts_out;

EigenSTL::vector_Vector3d acorn_db_slice_pts_loop;

void mesh_core::Mesh::slice(
      const Plane& plane,
      Mesh& a,
      Mesh& b) const
{
acorn_db_slice_current = 0;
acorn_db_slice_pts_clip.clear();
acorn_db_slice_pts_0.clear();
acorn_db_slice_pts_in.clear();
acorn_db_slice_pts_out.clear();

  std::vector<signed char> clipcodes;
  std::vector<double> dists;
  clipcodes.resize(verts_.size());
  dists.resize(verts_.size());

  int acnt = 0;
  int bcnt = 0;

  for (int i = 0 ; i < verts_.size() ; ++i)
  {
    dists[i] = plane.dist(verts_[i]);
    if (std::abs(dists[i]) < std::numeric_limits<double>::epsilon() /*  * 100 */)
    {
      clipcodes[i] = 0;
    }
    else if (dists[i] < 0.0)
    {
      acnt++;
      clipcodes[i] = 1;
    }
    else
    {
      bcnt++;
      clipcodes[i] = -1;
    }
  }

  a.clear();
  b.clear();

#if 0
  a.epsilon_ = b.epsilon_ = epsilon_;
  a.epsilon_squared_ = b.epsilon_squared_ = epsilon_squared_;
#endif

  if (bcnt == 0)
  {
    a = *this;
    return;
  }
  else if (acnt == 0)
  {
    b = *this;
    return;
  }

  a.reserve(tris_.size() * 2, verts_.size() * 2);
  b.reserve(tris_.size() * 2, verts_.size() * 2);

  std::vector<Triangle>::const_iterator tri = tris_.begin();
  std::vector<Triangle>::const_iterator tri_end = tris_.end();
  for ( ; tri != tri_end ; ++tri)
  {
    // ab is 0 for a and 1 for b
    for (int ab = 0 ; ab < 2 ; ++ab)
    {
      // ab_sign is 1 for a and -1 for b
      int ab_sign = ab ? -1 : 1;
      Mesh& newmesh = ab ? b : a;

if (ab==1)
{
  acorn_db_slice_current++;
  if (acorn_db_slice_current == acorn_db_slice_showclip)
  {
    acorn_db_slice_pts_clip.clear();
    acorn_db_slice_pts_0.clear();
    acorn_db_slice_pts_in.clear();
    acorn_db_slice_pts_out.clear();
  }
}


      // categorize triangle - inside, outside, or needs to be clipped?
      int cc[3];
      int in_cnt = 0;
      int out_cnt = 0;
      int start = -1;
      for (int i = 0 ; i < 3 ; ++i)
      {
        cc[i] = ab_sign * clipcodes[tri->verts_[i]];
        if (cc[i] > 0)
        {
          in_cnt++;
          start = i;
        }
        else if (cc[i] < 0)
        {
          out_cnt++;
        }
if (ab==1)
{
  if (acorn_db_slice_current == acorn_db_slice_showclip)
  {
    if (cc[i] == 0)
      acorn_db_slice_pts_0.push_back(verts_[tri->verts_[i]]);
    else if (cc[i] > 0)
      acorn_db_slice_pts_in.push_back(verts_[tri->verts_[i]]);
    else if (cc[i] < 0)
      acorn_db_slice_pts_out.push_back(verts_[tri->verts_[i]]);
  }
}

      }

      // entire triangle is out?
      if (in_cnt < 1)
      {
        continue;
      }

      // entire triangle is in?
      if (out_cnt < 1)
      {
        newmesh.add(
              verts_[tri->verts_[0]],
              verts_[tri->verts_[1]],
              verts_[tri->verts_[2]]);
        continue;
      }

      Eigen::Vector3d v[4];
      int vcnt = 0;

      ACORN_ASSERT(start >= 0);

      int prev;
      int cur = start;
      ACORN_ASSERT(cc[cur] > 0);
      while (cc[cur] >= 0)
      {
        v[vcnt++] = verts_[tri->verts_[cur]];
        prev = cur;
        cur = (cur + 1) % 3;
        ACORN_ASSERT(cur != start);
      }

      if (cc[prev] > 0)
      {
        // in->out transition
        int prev_idx = tri->verts_[prev];
        int cur_idx  = tri->verts_[cur];
        double t = dists[prev_idx] / (dists[prev_idx] - dists[cur_idx]);
        v[vcnt++] = (1.0 - t) * verts_[prev_idx] + t * verts_[cur_idx];


if (ab==1)
{
  if (acorn_db_slice_current == acorn_db_slice_showclip)
  {
    acorn_db_slice_pts_clip.push_back(v[vcnt-1]);

            acorn_db_slice_in = verts_[prev_idx];
            acorn_db_slice_out = verts_[cur_idx];
            acorn_db_slice_clip = v[vcnt-1];
            logInform("Clip %d:  t=%f = %f / ( %f - %f )   v[%d]   in->out",
              acorn_db_slice_current,
              t,
              dists[prev_idx],dists[prev_idx],dists[cur_idx],
              vcnt-1);
            logInform("     in: (%8.4f %8.4f %8.4f)",
              acorn_db_slice_in.x(),
              acorn_db_slice_in.y(),
              acorn_db_slice_in.z());
            logInform("    out: (%8.4f %8.4f %8.4f)",
              acorn_db_slice_out.x(),
              acorn_db_slice_out.y(),
              acorn_db_slice_out.z());
            logInform("    clp: (%8.4f %8.4f %8.4f)",
              acorn_db_slice_clip.x(),
              acorn_db_slice_clip.y(),
              acorn_db_slice_clip.z());
            logInform("    pln: (%8.4f %8.4f %8.4f) %8.4f",
              plane.getA(),
              plane.getB(),
              plane.getC(),
              plane.getD());
  }
}

      }

      while (cc[cur] < 0)
      {
        ACORN_ASSERT(cur != start);
        prev = cur;
        cur = (cur + 1) % 3;
      }

      if (cc[cur] > 0)
      {
        // out->in transition
        int prev_idx = tri->verts_[prev];
        int cur_idx  = tri->verts_[cur];
        double t = dists[cur_idx] / (dists[cur_idx] - dists[prev_idx]);
        v[vcnt++] = (1.0 - t) * verts_[cur_idx] + t * verts_[prev_idx];

if (ab==1)
{
  if (acorn_db_slice_current == acorn_db_slice_showclip)
  {
    acorn_db_slice_pts_clip.push_back(v[vcnt-1]);
            acorn_db_slice_in = verts_[cur_idx];
            acorn_db_slice_out = verts_[prev_idx];
            acorn_db_slice_clip = v[vcnt-1];
            logInform("Clip %d:  t=%f = %f / ( %f - %f )   v[%d]  out->in",
              acorn_db_slice_current,
              t,
              dists[cur_idx],dists[cur_idx],dists[prev_idx],
              vcnt-1);
            logInform("     in: (%8.4f %8.4f %8.4f)",
              acorn_db_slice_in.x(),
              acorn_db_slice_in.y(),
              acorn_db_slice_in.z());
            logInform("    out: (%8.4f %8.4f %8.4f)",
              acorn_db_slice_out.x(),
              acorn_db_slice_out.y(),
              acorn_db_slice_out.z());
            logInform("    clp: (%8.4f %8.4f %8.4f)",
              acorn_db_slice_clip.x(),
              acorn_db_slice_clip.y(),
              acorn_db_slice_clip.z());
            logInform("    pln: (%8.4f %8.4f %8.4f) %8.4f",
              plane.getA(),
              plane.getB(),
              plane.getC(),
              plane.getD());
  }
}


      }

      while (cur != start)
      {
        ACORN_ASSERT(cc[cur] >= 0);
        v[vcnt++] = verts_[tri->verts_[cur]];
        cur = (cur + 1) % 3;
      }
      
      ACORN_ASSERT(vcnt >= 3 && vcnt <= 4);

if (ab==1)
{
  if (acorn_db_slice_current == acorn_db_slice_showclip)
  {
            logInform("    ADD: tris.size=%d            (vcnt=%d)",newmesh.tris_.size(), vcnt);
            logInform("    ADD: (%8.4f %8.4f %8.4f) (%8.4f %8.4f %8.4f) (%8.4f %8.4f %8.4f)",
              v[0].x(),
              v[0].y(),
              v[0].z(),
              v[1].x(),
              v[1].y(),
              v[1].z(),
              v[2].x(),
              v[2].y(),
              v[2].z());
    acorn_debug_show_vertex_consolidate = true;
  }
}

      newmesh.add(v[0], v[1], v[2]);

if (ab==1)
{
  if (acorn_db_slice_current == acorn_db_slice_showclip)
  {
            logInform("     as: tris.size=%d",newmesh.tris_.size());
            logInform("     as: (%8.4f %8.4f %8.4f) (%8.4f %8.4f %8.4f) (%8.4f %8.4f %8.4f)",
              newmesh.verts_[newmesh.tris_[newmesh.tris_.size()-1].verts_[0]].x(),
              newmesh.verts_[newmesh.tris_[newmesh.tris_.size()-1].verts_[0]].y(),
              newmesh.verts_[newmesh.tris_[newmesh.tris_.size()-1].verts_[0]].z(),

              newmesh.verts_[newmesh.tris_[newmesh.tris_.size()-1].verts_[1]].x(),
              newmesh.verts_[newmesh.tris_[newmesh.tris_.size()-1].verts_[1]].y(),
              newmesh.verts_[newmesh.tris_[newmesh.tris_.size()-1].verts_[1]].z(),

              newmesh.verts_[newmesh.tris_[newmesh.tris_.size()-1].verts_[2]].x(),
              newmesh.verts_[newmesh.tris_[newmesh.tris_.size()-1].verts_[2]].y(),
              newmesh.verts_[newmesh.tris_[newmesh.tris_.size()-1].verts_[2]].z());
  }
}

      if (vcnt == 4)
      {

if (ab==1)
{
  if (acorn_db_slice_current == acorn_db_slice_showclip)
  {
            logInform("    ADD: (%8.4f %8.4f %8.4f) (%8.4f %8.4f %8.4f) (%8.4f %8.4f %8.4f)",
              v[0].x(),
              v[0].y(),
              v[0].z(),
              v[2].x(),
              v[2].y(),
              v[2].z(),
              v[3].x(),
              v[3].y(),
              v[3].z());
  }
}

        newmesh.add(v[0], v[2], v[3]);

if (ab==1)
{
  if (acorn_db_slice_current == acorn_db_slice_showclip)
  {
            logInform("     as: (%8.4f %8.4f %8.4f) (%8.4f %8.4f %8.4f) (%8.4f %8.4f %8.4f)",
              newmesh.verts_[newmesh.tris_[newmesh.tris_.size()-1].verts_[0]].x(),
              newmesh.verts_[newmesh.tris_[newmesh.tris_.size()-1].verts_[0]].y(),
              newmesh.verts_[newmesh.tris_[newmesh.tris_.size()-1].verts_[0]].z(),

              newmesh.verts_[newmesh.tris_[newmesh.tris_.size()-1].verts_[1]].x(),
              newmesh.verts_[newmesh.tris_[newmesh.tris_.size()-1].verts_[1]].y(),
              newmesh.verts_[newmesh.tris_[newmesh.tris_.size()-1].verts_[1]].z(),

              newmesh.verts_[newmesh.tris_[newmesh.tris_.size()-1].verts_[2]].x(),
              newmesh.verts_[newmesh.tris_[newmesh.tris_.size()-1].verts_[2]].y(),
              newmesh.verts_[newmesh.tris_[newmesh.tris_.size()-1].verts_[2]].z());
  }
}
      }
acorn_debug_show_vertex_consolidate = false;
    }
  }

  bool debug_this_node = debug_this_node_;
  debug_this_node_ = (debug_this_node && (debug_node_id_ & 1) == 0);

  a.fillGaps();
acorn_db_slice_pts_loop.clear();
  debug_this_node_ = (debug_this_node && (debug_node_id_ & 1) == 1);
  acorn_debug_current_node |= 1;
  b.fillGaps();
}

void mesh_core::Mesh::getBoundingSphere(
      Eigen::Vector3d& center,
      double &radius) const
{
  if (!bounding_sphere_valid_)
  {
    generateBoundingSphere(verts_, bounding_sphere_center_, bounding_sphere_radius_);
    bounding_sphere_valid_ = true;
  }
  center = bounding_sphere_center_;
  radius = bounding_sphere_radius_;
}

void mesh_core::Mesh::getAABB(
      Eigen::Vector3d& min,
      Eigen::Vector3d& max) const
{
  if (!aabb_valid_)
  {
    generateAABB(verts_, aabb_min_, aabb_max_);
    aabb_valid_ = true;
  }
  min = aabb_min_;
  max = aabb_max_;
}

const EigenSTL::vector_Vector3d& mesh_core::Mesh::getFaceNormals() const
{
  if (face_normals_.size() != tris_.size())
  {
    face_normals_.resize(tris_.size());

    std::vector<Triangle>::const_iterator tri = tris_.begin();
    std::vector<Triangle>::const_iterator end = tris_.end();
    for (int tidx = 0 ; tri != end ; ++tri, ++tidx)
    {
      face_normals_[tidx] = (verts_[tri->verts_[1]] - verts_[tri->verts_[0]]).cross(
                             verts_[tri->verts_[2]] - verts_[tri->verts_[0]]).normalized();
    }
  }
  return face_normals_;
}

void mesh_core::Mesh::deleteBioundingSphereTree(BoundingSphereNode *mesh_tree)
{
  if (!mesh_tree)
    return;

  BoundingSphereNode *sibling = mesh_tree->next_sibling_;
  if (sibling != mesh_tree)
  {
    mesh_tree->next_sibling_->prev_sibling_ = mesh_tree->prev_sibling_;
    mesh_tree->prev_sibling_->next_sibling_ = mesh_tree->next_sibling_;
    deleteBioundingSphereTree(sibling);
  }
  deleteBioundingSphereTree(mesh_tree->first_child_);
  delete mesh_tree->tmp_mesh_;
  delete mesh_tree;
}

// harvest spheres from leaves of tree
void mesh_core::Mesh::collectBoundingSpheres(
      BoundingSphereNode *mesh_tree,
      EigenSTL::vector_Vector3d& sphere_centers,
      std::vector<double>& sphere_radii,
      int max_depth)
{
  if (!mesh_tree)
    return;

  BoundingSphereNode *sibling = mesh_tree;
  do
  {
    if (sibling->first_child_ && max_depth != 0)
    {
      collectBoundingSpheres(sibling->first_child_, sphere_centers, sphere_radii, max_depth - 1);
    }
    else
    {
      Eigen::Vector3d center;
      double radius;
      sibling->mesh_->getBoundingSphere(center, radius);
      sphere_centers.push_back(center);
      sphere_radii.push_back(radius);
    }

    sibling = sibling->next_sibling_;
  }
  while (sibling != mesh_tree);
}

struct mesh_core::Mesh::BoundingSphereParams
{
  double tolerance_;
  int max_depth_;
};

void mesh_core::Mesh::getBoundingSpheres(
          double tolerance,
          EigenSTL::vector_Vector3d& sphere_centers,
          std::vector <double> sphere_radii,
          BoundingSphereNode **mesh_tree,
          int max_depth,
          SplitMethod split_method) const
{
  logInform("################### CALL getBoundingSpheres()");
  BoundingSphereNode *tree = NULL;
  sphere_centers.clear();
  sphere_radii.clear();

  if (tris_.size() > 0)
  {
    tree = new BoundingSphereNode;

    tree->mesh_ = this;
    tree->parent_ = NULL;
    tree->first_child_ = NULL;
    tree->next_sibling_ = tree;
    tree->prev_sibling_ = tree;
    tree->tmp_mesh_ = NULL;
    tree->depth_ = 0;
    tree->id_ = 1;
    tree->split_method_ = split_method;

    BoundingSphereParams params;
    params.tolerance_ = tolerance;
    params.max_depth_ = max_depth;
    
    calculateBoundingSpheres(params, tree);

    collectBoundingSpheres(tree, sphere_centers, sphere_radii);
  }

  if (mesh_tree)
    *mesh_tree = tree;
  else
    deleteBioundingSphereTree(tree);
}

// recursively divide into 2 pieces until the piece is fit well by a sphere
void mesh_core::Mesh::calculateBoundingSpheres(
          const BoundingSphereParams& params,
          BoundingSphereNode *mesh_node) const
{
  if (params.max_depth_ >= 0 && mesh_node->depth_ >= params.max_depth_)
    return;

  if (!calculateBoundingSpheresTreeSplit(params.tolerance_, mesh_node))
    return;

  ACORN_ASSERT(mesh_node->first_child_);
  ACORN_ASSERT(mesh_node->first_child_->next_sibling_);
  calculateBoundingSpheres(params, mesh_node->first_child_);
  calculateBoundingSpheres(params, mesh_node->first_child_->next_sibling_);
}

bool mesh_core::Mesh::calculateBoundingSpheresTreeSplit(
          double tolerance,
          BoundingSphereNode *mesh_node) const
{
  Mesh *mesh_a = NULL;
  Mesh *mesh_b = NULL;

  if (!calculateBoundingSpheresMeshSplit(tolerance, mesh_node, &mesh_a, &mesh_b))
    return false;

  BoundingSphereNode *a = new BoundingSphereNode;
  BoundingSphereNode *b = new BoundingSphereNode;

  a->mesh_ = mesh_a;
  a->tmp_mesh_ = mesh_a;
  a->parent_ = mesh_node;
  a->first_child_ = NULL;
  a->next_sibling_ = b;
  a->prev_sibling_ = b;
  a->depth_ = mesh_node->depth_ + 1;
  a->id_ = mesh_node->id_ << 1;
  a->split_method_ = mesh_node->split_method_;
  
  b->mesh_ = mesh_b;
  b->tmp_mesh_ = mesh_b;
  b->parent_ = mesh_node;
  b->first_child_ = NULL;
  b->next_sibling_ = a;
  b->prev_sibling_ = a;
  b->depth_ = mesh_node->depth_ + 1;
  b->id_ = a->id_ | 1;
  a->split_method_ = mesh_node->split_method_;
  
  ACORN_ASSERT(mesh_node->first_child_ == NULL);
  mesh_node->first_child_ = a;

  return true;
}

// If split needed, split mesh_node and return true.
// Else return false.
bool mesh_core::Mesh::calculateBoundingSpheresMeshSplit(
          double tolerance,
          BoundingSphereNode *mesh_node,
          Mesh **mesh_a,
          Mesh **mesh_b) const
{
  debug_this_node_ = (debug_node_id_ != -1 && ((debug_node_id_ & ~1) == (mesh_node->id_ << 1)));
  acorn_debug_current_node = mesh_node->id_<<1;

  Plane plane;
  if (!calculateBoundingSpheresSplitPlane(tolerance, mesh_node, plane))
    return false;

  *mesh_a = new Mesh;
  *mesh_b = new Mesh;

  mesh_node->mesh_->slice(plane, **mesh_a, **mesh_b);

  debug_this_node_ = false;

  mesh_node->plane_ = plane;

  if ((*mesh_a)->getTriCount() > 0 && (*mesh_b)->getTriCount() > 0)
    return true;

  logWarn("Splitting mesh id=%d depth=%d resulted in 0-triangle mesh.",
    mesh_node->id_,
    mesh_node->depth_);

  delete *mesh_a;
  delete *mesh_b;
  *mesh_a = NULL;
  *mesh_b = NULL;
  return false;
}

// If mesh_node should be split, calculate a splitting plane and return true.
// If no split needed, return false.
bool mesh_core::Mesh::calculateBoundingSpheresSplitPlane(
          double tolerance,
          BoundingSphereNode *mesh_node,
          Plane& plane) const
{
  switch (mesh_node->split_method_)
  {
  case SPLIT_CLOSE_FAR:
    return calculateBoundingSpheresSplitPlane_closeFar(tolerance, mesh_node, plane);
  case SPLIT_FAR:
    return calculateBoundingSpheresSplitPlane_far(tolerance, mesh_node, plane);
  case SPLIT_ORTHO:
    return calculateBoundingSpheresSplitPlane_ortho(tolerance, mesh_node, plane);
  case SPLIT_BIG_AXIS:
  default:
    return calculateBoundingSpheresSplitPlane_bigAxis(tolerance, mesh_node, plane);
  }
}

// If mesh_node should be split, calculate a splitting plane and return true.
// If no split needed, return false.
//
// This version works by splitting along greatest of x,y,z axis
bool mesh_core::Mesh::calculateBoundingSpheresSplitPlane_bigAxis(
          double tolerance,
          BoundingSphereNode *mesh_node,
          Plane& plane) const
{
  Eigen::Vector3d center;
  double radius;
  mesh_node->mesh_->getBoundingSphere(center, radius);

  // decide whether we need to split
  Eigen::Vector3d closest_point;
  int closest_tri;
  double closest_dist = findClosestPoint(
                            center, 
                            closest_point,
                            closest_tri);

  if (1) // DEBUG
  {
    mesh_node->closest_point = closest_point;
    const Triangle& tri = tris_[closest_tri];
    mesh_node->closes_triangle_.resize(3);
    mesh_node->closes_triangle_[0] = verts_[tri.verts_[0]];
    mesh_node->closes_triangle_[1] = verts_[tri.verts_[1]];
    mesh_node->closes_triangle_[2] = verts_[tri.verts_[2]];

    mesh_node->farthest_point_ = Eigen::Vector3d::Zero();
    mesh_node->plane_points_.clear();
  }

  // No need to split.  Sphere bounding is within tolerance.
  if (closest_dist + tolerance >= radius)
    return false;

  Eigen::Vector3d norm;
  Eigen::Vector3d min;
  Eigen::Vector3d max;
  mesh_node->mesh_->getAABB(min, max);
  Eigen::Vector3d size = max - min;
  if (size.x() > size.y())
  {
    if (size.x() > size.z())
    {
      norm = Eigen::Vector3d(1,0,0);
    }
    else
    {
      norm = Eigen::Vector3d(0,0,1);
    }
  }
  else if (size.y() > size.z())
  {
    norm = Eigen::Vector3d(0,1,0);
  }
  else
  {
    norm = Eigen::Vector3d(0,0,1);
  }

  plane = Plane(norm, (max + min) * 0.5);

  return true;
}

// If mesh_node should be split, calculate a splitting plane and return true.
// If no split needed, return false.
//
// This version works by:
//   1) if parents, splitting plane kis perpendicular to parent(s)
//   2) otherwise try to use longest axis
bool mesh_core::Mesh::calculateBoundingSpheresSplitPlane_ortho(
          double tolerance,
          BoundingSphereNode *mesh_node,
          Plane& plane) const
{
  Eigen::Vector3d center;
  double radius;
  mesh_node->mesh_->getBoundingSphere(center, radius);

  // decide whether we need to split
  Eigen::Vector3d closest_point;
  int closest_tri;
  double closest_dist = findClosestPoint(
                            center, 
                            closest_point,
                            closest_tri);

  if (1) // DEBUG
  {
    mesh_node->closest_point = closest_point;
    const Triangle& tri = tris_[closest_tri];
    mesh_node->closes_triangle_.resize(3);
    mesh_node->closes_triangle_[0] = verts_[tri.verts_[0]];
    mesh_node->closes_triangle_[1] = verts_[tri.verts_[1]];
    mesh_node->closes_triangle_[2] = verts_[tri.verts_[2]];

    mesh_node->farthest_point_ = Eigen::Vector3d::Zero();
    mesh_node->plane_points_.clear();
  }

  // No need to split.  Sphere bounding is within tolerance.
  if (closest_dist + tolerance >= radius)
    return false;


  if (mesh_node->parent_ && mesh_node->parent_->parent_)
  {
    Eigen::Vector3d norm;
    norm = mesh_node->parent_->plane_.getNormal().cross(
           mesh_node->parent_->parent_->plane_.getNormal());

    norm.normalize();

    double max_d = -std::numeric_limits<double>::max();
    double min_d =  std::numeric_limits<double>::max();
    EigenSTL::vector_Vector3d::const_iterator it = mesh_node->mesh_->verts_.begin();
    EigenSTL::vector_Vector3d::const_iterator end = mesh_node->mesh_->verts_.end();
    for ( ; it != end ; ++it)
    {
      double d = -norm.dot(*it);
      max_d = std::max(max_d, d);
      min_d = std::min(min_d, d);
    }

    plane = Plane(norm, (max_d + min_d) * 0.5);
    return true;
  }

  Eigen::Vector3d norm;
  Eigen::Vector3d min;
  Eigen::Vector3d max;
  mesh_node->mesh_->getAABB(min, max);
  Eigen::Vector3d size = max - min;
  if (size.x() > size.y())
  {
    if (size.x() > size.z())
    {
      norm = Eigen::Vector3d(1,0,0);
    }
    else
    {
      norm = Eigen::Vector3d(0,0,1);
    }
  }
  else if (size.y() > size.z())
  {
    norm = Eigen::Vector3d(0,1,0);
  }
  else
  {
    norm = Eigen::Vector3d(0,0,1);
  }

  if (mesh_node->parent_)
  {
    const Eigen::Vector3d& parent_norm = mesh_node->parent_->plane_.getNormal();
    Eigen::Vector3d tmp = norm.cross(parent_norm).cross(parent_norm);
    if (tmp.squaredNorm() <= std::numeric_limits<double>::epsilon())
      tmp = parent_norm.cross(Eigen::Vector3d(parent_norm.y(), parent_norm.z(), parent_norm.x()));
    norm = tmp;
  }

  plane = Plane(norm, (max + min) * 0.5);

  return true;
}

// If mesh_node should be split, calculate a splitting plane and return true.
// If no split needed, return false.
//
// This version works by:
//   1) split plane goes through center of split-mesh bounding sphere.
//   2) find farthest vertex from center.  Plane is perpendicular to center-farpoint
bool mesh_core::Mesh::calculateBoundingSpheresSplitPlane_far(
          double tolerance,
          BoundingSphereNode *mesh_node,
          Plane& plane) const
{
  const Mesh& mesh_node_mesh = *mesh_node->mesh_;
  int node_nverts = mesh_node_mesh.verts_.size();
  ACORN_ASSERT(node_nverts >= 3);

  Eigen::Vector3d center;
  double radius;
  mesh_node_mesh.getBoundingSphere(center, radius);

  // decide whether we need to split
  Eigen::Vector3d closest_point;
  int closest_tri;
  double closest_dist = findClosestPoint(
                            center, 
                            closest_point,
                            closest_tri);
  if (1) // DEBUG
  {
    mesh_node->closest_point = closest_point;
    const Triangle& tri = tris_[closest_tri];
    mesh_node->closes_triangle_.resize(3);
    mesh_node->closes_triangle_[0] = verts_[tri.verts_[0]];
    mesh_node->closes_triangle_[1] = verts_[tri.verts_[1]];
    mesh_node->closes_triangle_[2] = verts_[tri.verts_[2]];

    mesh_node->farthest_point_ = Eigen::Vector3d::Zero();
    mesh_node->plane_points_.clear();
  }

  // No need to split.  Sphere bounding is within tolerance.
  if (closest_dist + tolerance >= radius)
    return false;


  // find mesh vertex farthest from center of bounding sphere
  int far_vidx = 0;
  double far_vidx_dsq = -1.0;
  for (int i = 0; i < node_nverts ; i++)
  {
    double dsq = (mesh_node_mesh.verts_[i] - center).squaredNorm();
    if (dsq < far_vidx_dsq)
    {
      far_vidx_dsq = dsq;
      far_vidx = i;
    }
  }

  if (1) // DEBUG
  {
    mesh_node->farthest_point_ = mesh_node_mesh.verts_[far_vidx];
  }

  Eigen::Vector3d norm = mesh_node_mesh.verts_[far_vidx] - center;
  if (norm.squaredNorm() < tolerance*tolerance)
    return false;

  plane = Plane(norm, center);
  return true;
}

// If mesh_node should be split, calculate a splitting plane and return true.
// If no split needed, return false.
//
// This version works by:
//   1) split plane goes through center of split-mesh bounding sphere.
//   2) find closest point on original mesh to center of split-mesh bounding sphere.  It will be on split plane.
//   3) find farthest point from the line of points 1 and 1.  Plane avoids this point.
bool mesh_core::Mesh::calculateBoundingSpheresSplitPlane_closeFar(
          double tolerance,
          BoundingSphereNode *mesh_node,
          Plane& plane) const
{
  int nverts = verts_.size();
  int ntris = tris_.size();
  ACORN_ASSERT(nverts >= 3);


  Eigen::Vector3d center;
  double radius;
  mesh_node->mesh_->getBoundingSphere(center, radius);

  Eigen::Vector3d closest_point;
  int closest_tri;
  double closest_dist = findClosestPoint(
                            center, 
                            closest_point,
                            closest_tri);

  // debug
  if (1)
  {
    mesh_node->closest_point = closest_point;
    const Triangle& tri = tris_[closest_tri];
    mesh_node->closes_triangle_.resize(3);
    mesh_node->closes_triangle_[0] = verts_[tri.verts_[0]];
    mesh_node->closes_triangle_[1] = verts_[tri.verts_[1]];
    mesh_node->closes_triangle_[2] = verts_[tri.verts_[2]];
  }

  // No need to split.  Sphere bounding is within tolerance.
  if (closest_dist + tolerance >= radius)
    return false;

#if 0
  // find nearby points that are also close to center of sphere
  Eigen::Vector3d closest2_point = closest_point;
  Eigen::Vector3d closest3_point = closest_point;
  double closest2_dist = std::numeric_limits<double>::max();
  double closest3_dist = std::numeric_limits<double>::max();
  int closest2_tri = -1;
  int closest3_tri = -1;

  static const double EPSILON = std::numeric_limits<double>::epsilon() * 100;
  static const double EPSILON_SQ = EPSILON * EPSILON;

  {
    std::set<int> tris_set;   // which triangles we have added already
    std::vector<int> tris1;
    std::vector<int> tris2;

    tris_set.insert(closest_tri);

    // initialize tris1 with all triangles adjacent to the closest tri
    for (int i = 0 ; i < 3 ; ++i)
    {
      const Edge& edge = edges_[tris_[closest_tri].edges_[i].edge_idx_];
      int netris = edge.tris_.size();
      for (int j = 0 ; j < netris ; ++j)
      {
        int tidx = edge.tris_[j].tri_idx_;
        if (!tris_set.insert(tidx).second)
          tris1.push_back(tidx);    // not seen yet; add it
      }
    }

    for (;;)
    {
      while (!tris1.empty())
      {
        int tidx = tris1.back();
        tris1.pop_back();

        const Triangle& tri = tris_[tidx];
        Eigen::Vector3d close_point;
        double dist = closestPointOnTriangle(
                          verts_[tri.verts_[0]],
                          verts_[tri.verts_[1]],
                          verts_[tri.verts_[2]],
                          center,
                          close_point,
                          closest2_dist);
        Eigen::Vector3d edgevec_a = close_point - closest_point;
        Eigen::Vector3d edgevec_b = close_point - closest2_point;
        Eigen::Vector3d edgevec_c = close_point - closest3_point;
        if (dist < closest2_dist)
        {
          if (edgevec_a.squaredNorm() > EPSILON_SQ)
          {
            if (closest2_tri == -1)
            {
              closest2_dist = dist;
              closest2_point = close_point;
              closest2_tri = tidx;
            }
            else if (edgevec_b.squaredNorm() > EPSILON_SQ &&
                     edgevec_a.cross(edgevec_b).squaredNorm() > EPSILON_SQ)
            {
              closest3_dist = closest2_dist;
              closest3_point = closest2_point;
              closest3_tri = closest2_tri;

              closest2_dist = dist;
              closest2_point = close_point;
              closest2_tri = tidx;
            }
            else if (edgevec_c.squaredNorm() > EPSILON_SQ &&
                     edgevec_a.cross(edgevec_c).squaredNorm() > EPSILON_SQ)
            {
              closest2_dist = dist;
              closest2_point = close_point;
              closest2_tri = tidx;
            }
          }
        }
        else if (dist < closest3_dist)
        {
          if (edgevec_a.squaredNorm() > EPSILON_SQ &&
              edgevec_b.squaredNorm() > EPSILON_SQ &&
              edgevec_a.cross(edgevec_b).squaredNorm() > EPSILON_SQ)
          {
            closest3_dist = dist;
            closest3_point = close_point;
            closest3_tri = tidx;
          }
        }

        for (int i = 0 ; i < 3 ; ++i)
        {
          const Edge& edge = edges_[tri.edges_[i].edge_idx_];
          int netris = edge.tris_.size();
          for (int j = 0 ; j < netris ; ++j)
          {
            int tidx = edge.tris_[j].tri_idx_;
            if (!tris_set.insert(tidx).second)
              tris2.push_back(tidx);  // not seen yet; add it
          }
        }
      }

      // found 3 orthogonal points?
      if (closest3_tri != -1)
        break;

      // no more points to consider?
      if (tris2.empty())
        break;
      
      // try the next set of adjacent triangles
      std::swap(tris1, tris2);
    }
  }


  EigenSTL::vector_Vector3d points;
  points.reserve(4);
  points.push_back(center);

  points.push_back(closest_point);
  if (closest2_tri >=0)
    points.push_back(closest2_point);
  if (closest3_tri >=0)
    points.push_back(closest3_point);
  while (points.size() < 3)
    points.push_back(center);

#else
  // split line is line from closest_point to center.  Need to find split plane
  // containing that line
  Eigen::Vector3d x_axis = closest_point - center;
  if (x_axis.squaredNorm() <  std::numeric_limits<double>::epsilon())
    x_axis = Eigen::Vector3d(1,0,0);

  PlaneProjection proj(x_axis, center);

  // find farthest point from split line
  const Mesh& mesh_node_mesh = *mesh_node->mesh_;
  int node_nverts = mesh_node_mesh.verts_.size();
  int far_vidx = 0;
  double far_vidx_dsq = -1.0;
  for (int i = 0; i < node_nverts ; i++)
  {
    Eigen::Vector2d p = proj.project(mesh_node_mesh.verts_[i]);

    double dsq = p.squaredNorm();
    if (dsq > far_vidx_dsq)
    {
      far_vidx_dsq = dsq;
      far_vidx = i;
    }
  }

  // debug
  mesh_node->farthest_point_ = mesh_node_mesh.verts_[far_vidx];

  // y_axis is another line that should be in the plane
  Eigen::Vector3d y_axis = (mesh_node_mesh.verts_[far_vidx] - center).cross(x_axis);


  EigenSTL::vector_Vector3d points;
  points.reserve(4);
  points.push_back(center);
  points.push_back(closest_point);
  points.push_back(center + y_axis);
#endif

  // debug
  mesh_node->plane_points_ = points;

#if 0
  plane = Plane(points);
#else
  plane.from3Points(points);
#endif

  return true;
}


// find point on surface of mesh closest to given point.
// return closest_point which is on mesh.
// return distance from point to that point.
// return index of triangle 
double mesh_core::Mesh::findClosestPoint(
      const Eigen::Vector3d& point,
      Eigen::Vector3d& closest_point,
      int& closest_tri) const
{
  int nverts = verts_.size();
  int ntris = tris_.size();
  ACORN_ASSERT(nverts >= 3);

  if (ntris < 0)
  {
    logError("findClosestPoint() called on empty mesh.");
    closest_tri = -1;
    closest_point = point;
    return 0.0;
  }

  // find mesh vertex closest to point
  int close_vidx = 0;
  double close_vidx_dsq = std::numeric_limits<double>::max();
  for (int i = 0; i < nverts ; i++)
  {
    double dsq = (verts_[i] - point).squaredNorm();
    if (dsq < close_vidx_dsq)
    {
      close_vidx_dsq = dsq;
      close_vidx = i;
    }
  }

  // find point on mesh closest to point
  closest_point = verts_[close_vidx];
  ACORN_ASSERT(vert_info_.size() > close_vidx);
  ACORN_ASSERT(vert_info_[close_vidx].edges_.size() > 0);
  ACORN_ASSERT(edges_[vert_info_[close_vidx].edges_[0]].tris_.size() > 0);
  closest_tri = edges_[vert_info_[close_vidx].edges_[0]].tris_[0].tri_idx_;
  double closest_dist = std::sqrt(close_vidx_dsq);
  for (int i = 0 ; i < ntris ; ++i)
  {
    const Triangle& tri = tris_[i];
    Eigen::Vector3d close_point;
    double dist = closestPointOnTriangle(
                      verts_[tri.verts_[0]],
                      verts_[tri.verts_[1]],
                      verts_[tri.verts_[2]],
                      point,
                      close_point,
                      closest_dist);
    if (dist < closest_dist)
    {
      closest_dist = dist;
      closest_point = close_point;
      closest_tri = i;
    }
  }

  return closest_dist;
}

void mesh_core::Mesh::addSphere(
      const Eigen::Vector3d& center,
      double radius,
      double max_error)
{
  static Eigen::Vector3d icosa_vecs[10];
  static const Eigen::Vector3d icosa_top(0,1,0);
  static const Eigen::Vector3d icosa_bot(0,-1,0);
  static bool initialized = false;

  if (!initialized)
  {
    const double mid_lat = std::atan(0.5);
    const double y = sin(mid_lat);
    const double xz = cos(mid_lat);
    for (int i = 0 ; i < 10 ; ++i)
    {
      double a = 2.0 * boost::math::constants::pi<double>() * i / 10.0;
      double x = xz * cos(a);
      double z = xz * sin(a);
      icosa_vecs[i] = Eigen::Vector3d(x, (i&1)?-y:y, z);
    }
    initialized = true;
  }
  
  if (max_error < std::numeric_limits<double>::epsilon() * 10.0)
    max_error = std::numeric_limits<double>::epsilon() * 10.0;

  // top of icosahedron
  for (int i = 0 ; i < 5 ; i++)
  {
    addSphereTri(
              center,
              radius,
              max_error,
              icosa_top,
              icosa_vecs[(i * 2 + 2) % 10],
              icosa_vecs[i * 2],
              0);
  }

  // bottom of icosahedron
  for (int i = 0 ; i < 5 ; i++)
  {
    addSphereTri(
              center,
              radius,
              max_error,
              icosa_bot,
              icosa_vecs[i * 2 + 1],
              icosa_vecs[(i * 2 + 3) % 10],
              0);
  }

  // sides of icosahedron
  for (int i = 0 ; i < 5 ; i++)
  {
    addSphereTri(
              center,
              radius,
              max_error,
              icosa_vecs[i * 2],
              icosa_vecs[(i * 2 + 2) % 10],
              icosa_vecs[i * 2 + 1],
              0);
    addSphereTri(
              center,
              radius,
              max_error,
              icosa_vecs[i * 2 + 1],
              icosa_vecs[(i * 2 + 2) % 10],
              icosa_vecs[(i * 2 + 3) % 10],
              0);
  }
}

void mesh_core::Mesh::addSphereTri(
      const Eigen::Vector3d& center,
      double radius,
      double max_error,
      const Eigen::Vector3d& a,
      const Eigen::Vector3d& b,
      const Eigen::Vector3d& c,
      int depth)
{
  Eigen::Vector3d mid = (a + b + c) / 3.0;
  Eigen::Vector3d mid_norm = mid.normalized();
  double dsq = (mid - mid_norm).squaredNorm();
  if (dsq * radius * radius <= max_error * max_error)
  {
    add(
      center + a * radius,
      center + b * radius,
      center + c * radius);
    return;
  }

  Eigen::Vector3d ab = (a + b).normalized();
  Eigen::Vector3d ac = (a + c).normalized();
  Eigen::Vector3d bc = (b + c).normalized();

  addSphereTri(center, radius, max_error, a, ab, ac, depth + 1);
  addSphereTri(center, radius, max_error, ab, b, bc, depth + 1);
  addSphereTri(center, radius, max_error, ab, bc, ac, depth + 1);
  addSphereTri(center, radius, max_error, ac, bc, c, depth + 1);
}

struct ScanTri
{
  Eigen::Vector3d norm_;       // face normal
  Eigen::Vector3d edges_[3];   // edges_[n].dot(x,y,1) is positive if x,y is outside triangle
  Eigen::Vector3d zfunc_;      // z = zfunc_.dot(x,y,1)

  Eigen::Vector2d min_;
  Eigen::Vector2d max_;

  int tri_idx_;
};

bool ACORN_DEBUG_MESH_getInsidePoints = false;

void mesh_core::Mesh::getInsidePoints(
      EigenSTL::vector_Vector3d& points,
      double resolution) const
{
  points.clear();
  if (resolution <= std::numeric_limits<double>::epsilon())
    return;

  double oo_resolution = 1.0 / resolution;

  const Edge* gap_edge = findGap();
  ACORN_ASSERT(!gap_edge);
  if (gap_edge)
  {
    logError("ERROR: mesh_core::Mesh::getInsidePoints() called on mesh with gaps.  Call fillGaps() first.");
    return;
  }

  Eigen::Vector3d min;
  Eigen::Vector3d max;
  getAABB(min, max);

  Eigen::Vector3d origin;
  Eigen::Vector3i isize;

  for (int i = 0 ; i < 3 ; ++i)
  {
    int minval = int(std::floor(min(i) * oo_resolution));
    origin(i) = minval * resolution;
    int maxval = int(std::ceil((max(i) - origin(i)) * oo_resolution));
    isize(i) = maxval + 1;
  }

  points.reserve(isize.x() * isize.y() * isize.z());

  if (ACORN_DEBUG_MESH_getInsidePoints)
  {
    print();
    logInform("############ CALLING getInsidePoints() ntris=%d isize=(%d, %d, %d)",
      int(tris_.size()),
      isize.x(),
      isize.y(),
      isize.z());
    logInform("       org=(%8.4f, %8.4f, %8.4f)",
      origin.x(),
      origin.y(),
      origin.z());
    logInform("       min=(%8.4f, %8.4f, %8.4f)",
      min.x(),
      min.y(),
      min.z());
    logInform("       max=(%8.4f, %8.4f, %8.4f)",
      max.x(),
      max.y(),
      max.z());
  }

  // list of ScnTri, one for each triangle in mesh
  std::vector<ScanTri> tris;
  std::multimap<double, ScanTri*> x_coming; // triangles we have not seen yet, sorted by min x
  std::multimap<double, ScanTri*> x_current; // current triangles, sorted by max x

  // create a ScanTri for each triangle in the mesh.
  // Skip triangles perpendicular to z axis
  tris.resize(tris_.size());
  std::vector<Triangle>::const_iterator tri = tris_.begin();
  std::vector<Triangle>::const_iterator tri_end = tris_.end();
  int t = 0;
  for (int tri_idx = 0 ; tri != tri_end ; ++tri, ++tri_idx)
  {
    ScanTri& stri = tris[t];
    const Eigen::Vector3d& a = verts_[tri->verts_[0]];
    const Eigen::Vector3d& b = verts_[tri->verts_[1]];
    const Eigen::Vector3d& c = verts_[tri->verts_[2]];

    Eigen::Vector3d ab = b - a;
    Eigen::Vector3d ac = c - a;
    stri.norm_ = ab.cross(ac).normalized();
    if (std::abs(stri.norm_.z()) < std::numeric_limits<double>::epsilon())
    {
      if (ACORN_DEBUG_MESH_getInsidePoints)
      {
        logInform("  eliminate tri[%d] (norm.z = %f)", tri_idx, stri.norm_.z());
      }
      
      continue;
    }

    stri.tri_idx_ = tri_idx;

    double ooz = 1.0 / stri.norm_.z();
    double d = -stri.norm_.dot(a);
    stri.zfunc_.x() = -stri.norm_.x() * ooz;
    stri.zfunc_.y() = -stri.norm_.y() * ooz;
    stri.zfunc_.z() = -d * ooz;

    stri.min_.x() = std::min(std::min(a.x(), b.x()), c.x());
    stri.min_.y() = std::min(std::min(a.y(), b.y()), c.y());
    stri.max_.x() = std::max(std::max(a.x(), b.x()), c.x());
    stri.max_.y() = std::max(std::max(a.y(), b.y()), c.y());

    for (int i = 0 ; i < 3 ; ++i)
    {
      const Eigen::Vector3d& v0 = verts_[tri->verts_[i]];
      const Eigen::Vector3d& v1 = verts_[tri->verts_[(i+1)%3]];
      Eigen::Vector3d vec = v1 - v0;
      if (stri.norm_.z() < 0.0)
      {
        stri.edges_[i].x() = -vec.y();
        stri.edges_[i].y() =  vec.x();
      }
      else
      {
        stri.edges_[i].x() =  vec.y();
        stri.edges_[i].y() = -vec.x();
      }
      double d = stri.edges_[i].x() * v0.x() + stri.edges_[i].y() * v0.y();
      stri.edges_[i].z() = -d;
    }

    x_coming.insert(std::pair<double, ScanTri*>(stri.min_.x(), &stri));
    t++;
  }

  if (ACORN_DEBUG_MESH_getInsidePoints)
  {
    logInform("  %d tris after elimination", x_coming.size());
  }

  // iterate over all x,y positions.  For each one trace a ray (aka scanline)
  // parallel to the z axis to see what triangles we hit.  Points inside are
  // added to the points vector.
  Eigen::Vector3d coord(origin.x(), origin.y(), 1.0);
  for (int ix = 0 ; ix < isize.x() ; ++ix, coord.x() += resolution)
  {
    coord.y() = origin.y();

    if (ACORN_DEBUG_MESH_getInsidePoints)
    {
      logInform("x=%4d (%8.4f) ntris=%d",ix,coord.x(), x_current.size());
    }

    // add new triangles
    while (!x_coming.empty())
    {
      std::multimap<double, ScanTri*>::iterator it = x_coming.begin();
      if (it->first > coord.x())
        break;
      if (ACORN_DEBUG_MESH_getInsidePoints)
      {
        logInform("  (x) add    tri[%4d]\n%s\n%s\n%s",
          it->second->tri_idx_,
          str(verts_[tris_[it->second->tri_idx_].verts_[0]]).c_str(),
          str(verts_[tris_[it->second->tri_idx_].verts_[1]]).c_str(),
          str(verts_[tris_[it->second->tri_idx_].verts_[2]]).c_str());
      }
      x_current.insert(std::pair<double, ScanTri*>(it->second->max_.x(), it->second));
      x_coming.erase(it);
    }

    // remove old triangles
    while (!x_current.empty())
    {
      std::multimap<double, ScanTri*>::iterator it = x_current.begin();
      if (it->first >= coord.x())
        break;

      if (ACORN_DEBUG_MESH_getInsidePoints)
      {
        logInform("  (x) remove tri[%4d]\n%s\n%s\n%s",
          it->second->tri_idx_,
          str(verts_[tris_[it->second->tri_idx_].verts_[0]]).c_str(),
          str(verts_[tris_[it->second->tri_idx_].verts_[1]]).c_str(),
          str(verts_[tris_[it->second->tri_idx_].verts_[2]]).c_str());
      }
      x_current.erase(it);
    }

    if (x_current.empty())
      continue;

    std::multimap<double, ScanTri*> y_coming; // triangles we have not seen yet, sorted by min y
    std::multimap<double, ScanTri*> y_current; // current triangles, sorted by max y
    std::multimap<double, ScanTri*>::iterator xit = x_current.begin();
    std::multimap<double, ScanTri*>::iterator xit_end = x_current.end();
    for ( ; xit != xit_end ; ++xit)
    {
      y_coming.insert(std::pair<double, ScanTri*>(xit->second->min_.y(), xit->second));
    }

    for (int iy = 0 ; iy < isize.y() ; ++iy, coord.y() += resolution)
    {
      if (ACORN_DEBUG_MESH_getInsidePoints)
      {
        logInform("  xy=(%4d %4d) = (%8.4f %8.4f) ntris=%d",ix,iy,coord.x(),coord.y(), y_current.size());
      }

      // add new triangles
      while (!y_coming.empty())
      {
        std::multimap<double, ScanTri*>::iterator it = y_coming.begin();
        if (it->first > coord.y())
          break;
        if (ACORN_DEBUG_MESH_getInsidePoints)
        {
          logInform("    (y) add    tri[%4d]\n%s\n%s\n%s",
            it->second->tri_idx_,
            str(verts_[tris_[it->second->tri_idx_].verts_[0]]).c_str(),
            str(verts_[tris_[it->second->tri_idx_].verts_[1]]).c_str(),
            str(verts_[tris_[it->second->tri_idx_].verts_[2]]).c_str());
        }
        y_current.insert(std::pair<double, ScanTri*>(it->second->max_.y(), it->second));
        y_coming.erase(it);
      }

      // remove old triangles
      while (!y_current.empty())
      {
        std::multimap<double, ScanTri*>::iterator it = y_current.begin();
        if (it->first >= coord.y())
          break;
        if (ACORN_DEBUG_MESH_getInsidePoints)
        {
          logInform("    (y) remove tri[%4d]\n%s\n%s\n%s",
            it->second->tri_idx_,
            str(verts_[tris_[it->second->tri_idx_].verts_[0]]).c_str(),
            str(verts_[tris_[it->second->tri_idx_].verts_[1]]).c_str(),
            str(verts_[tris_[it->second->tri_idx_].verts_[2]]).c_str());
        }
        y_current.erase(it);
      }

      std::multimap<double, ScanTri*> scanline; // triangles on this scanline

      // find triangles on this xy scanline
      std::multimap<double, ScanTri*>::iterator yit = y_current.begin();
      std::multimap<double, ScanTri*>::iterator yit_end = y_current.end();
      for ( ; yit != yit_end ; ++yit)
      {
        for (int i = 0 ;; ++i)
        {
          // not outside any edge :: scanline hits triangle
          if (i == 3)
          {
            double z = coord.dot(yit->second->zfunc_);
            scanline.insert(std::pair<double, ScanTri*>(z, yit->second));
            if (ACORN_DEBUG_MESH_getInsidePoints)
            {
              logInform("       (z) scanline hit tri[%4d] at (%8.4f, %8.4f, %8.4f) norm.z=%8.4f",
                yit->second->tri_idx_,coord.x(), coord.y(), z, yit->second->norm_.z());
            }
            break;
          }

          if (ACORN_DEBUG_MESH_getInsidePoints)
          {
            logInform("                          check tri[%4d] edge[%d] dot=%f  c=%s e=%s",
              yit->second->tri_idx_,
              i,
              coord.dot(yit->second->edges_[i]),
              mesh_core::str(coord).c_str(),
              mesh_core::str(yit->second->edges_[i]).c_str());
          }

          // scanline does not hit this triangle?
          if (coord.dot(yit->second->edges_[i]) > 0.0)
            break;
        }
      }

      if (scanline.empty())
        continue;

      // walk the scanline and insert points
      Eigen::Vector3d pos(coord.x(), coord.y(), origin.z());
      int in = 0;
      int out = 0;
      double last_in_z = -std::numeric_limits<double>::max();
      double last_out_z = -std::numeric_limits<double>::max();
      for (int iz = 0 ; iz < isize.z() ; ++iz, pos.z() += resolution)
      {
        // remove triangles before this z value
        while (!scanline.empty())
        {
          std::multimap<double, ScanTri*>::iterator it = scanline.begin();
          if (it->first >= pos.z())
            break;

          if (it->second->norm_.z() < 0.0)
          {
            // avoid incrementing twice if on edge/vert shared by multiple triangles.
            if (std::abs(it->first - last_in_z) > std::numeric_limits<double>::epsilon())
            {
              ++in;
              last_in_z = it->first;
            }

            if (ACORN_DEBUG_MESH_getInsidePoints)
              logInform("         at z=%8.4f  in=%d out=%d",pos.z(), in, out);

          }
          else
          {
            // avoid incrementing twice if on edge/vert shared by multiple triangles.
            if (std::abs(it->first - last_out_z) > std::numeric_limits<double>::epsilon())
            {
              ++out;
              last_out_z = it->first;
            }

            if (ACORN_DEBUG_MESH_getInsidePoints)
              logInform("         at z=%8.4f  in=%d out=%d",pos.z(), in, out);
          }
          scanline.erase(it);
        }
        if (scanline.empty())
        {
          if (in != out)
            logWarn("getInsidePoints() scanline ends with in=%d != out=%d",in,out);
          break;
        }

        if (out > in || in > out+1)
        {
          logWarn("getInsidePoints() scanline has in=%d != out=%d",in,out);
        }

        if (in > out)
          points.push_back(pos);
      }
    }
  }
}


void mesh_core::Mesh::findThinnestFeature(
      double& dist,
      int& tria,
      int& trib,
      double max_angle) const
{
  dist = std::numeric_limits<double>::max();
  tria = -1;
  trib = -1;

  getFaceNormals();

  max_angle = std::abs(max_angle);
  if (max_angle > 85.0)
    max_angle = 85.0;
  double max_cos = -std::cos(max_angle * boost::math::constants::pi<double>() / 180.0);

  for (int a = tris_.size() - 1 ; a >= 0 ; --a)
  {
    const Eigen::Vector3d& anorm = face_normals_[a];
    const Triangle& tri_a = tris_[a];
    for (int b = a - 1 ; b >= 0 ; --b)
    {
      if (anorm.dot(face_normals_[b]) > max_cos)
        continue;

      const Triangle& tri_b = tris_[b];

      double best_d = dist;
      for (int i = 0 ; i < 3 ; ++i)
      {
        Eigen::Vector3d closest_point;
        double d = closestPointOnTriangle(
                                  verts_[tri_a.verts_[0]],
                                  verts_[tri_a.verts_[1]],
                                  verts_[tri_a.verts_[2]],
                                  verts_[tri_b.verts_[i]],
                                  closest_point,
                                  best_d);
        best_d = std::min(best_d, d);
        d = closestPointOnTriangle(
                                  verts_[tri_b.verts_[0]],
                                  verts_[tri_b.verts_[1]],
                                  verts_[tri_b.verts_[2]],
                                  verts_[tri_a.verts_[i]],
                                  closest_point,
                                  best_d);
        best_d = std::min(best_d, d);
      }

      if (best_d >= dist)
        continue;

      // skip if they share a vertex
      for (int i = 0 ; i < 3 ; ++i)
      {
        for (int j = 0 ; j < 3 ; ++j)
        {
          if (tri_a.verts_[i] == tri_b.verts_[j])
            goto do_continue;
        }
      }

      dist = best_d;
      tria = a;
      trib = b;

      do_continue:
      (void)0;
    }
  }
}
