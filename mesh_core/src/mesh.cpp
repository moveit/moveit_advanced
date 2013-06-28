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
  , number_of_submeshes_(-1)
  , adjacent_tris_valid_(false)
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
  t.submesh_ = -1;

  number_of_submeshes_ = -1; // need to recalculate
  adjacent_tris_valid_ = false;

  // find 3 edges
  for (int i = 0; i < 3 ; ++i)
  {
    t.adjacent_tris_[i] = -1;
    t.adjacent_tris_back_dir_[i] = -1;
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

void mesh_core::Mesh::setAdjacentTriangles()
{
  if (adjacent_tris_valid_)
    return;

  std::vector<Triangle>::iterator it = tris_.begin();
  std::vector<Triangle>::iterator end = tris_.end();
  for (int tidx = 0 ; it != end ; ++it, ++tidx)
  {
    for (int dir = 0 ; dir < 3 ; ++dir)
    {
      it->adjacent_tris_[dir] = -1;
      it->adjacent_tris_back_dir_[dir] = -1;

      int edge_idx = it->edges_[dir];
      Edge& edge = edges_[edge_idx];
      if (edge.tris_[0] == -2)
      {
        it->adjacent_tris_[dir] = -2;
        continue;
      }
      if (edge.tris_[0] == -1)
      {
        logError("setAdjacentTriangles found edge.tris_[0] == -1");
        it->adjacent_tris_[dir] = -1;
        continue;
      }
      if (edge.tris_[1] == -1)
      {
        it->adjacent_tris_[dir] = -1;
        continue;
      }

      int tadj_idx = edge.tris_[0] == tidx ?
                     edge.tris_[1] :
                     edge.tris_[0];
      Triangle& tadj = tris_[tadj_idx];

      for (int adj_dir = 0 ; adj_dir < 3 ; ++adj_dir)
      {
        if (tadj.edges_[adj_dir] == edge_idx)
        {
          it->adjacent_tris_[dir] = tadj_idx;
          tadj.adjacent_tris_[adj_dir] = tidx;
          it->adjacent_tris_back_dir_[dir] = adj_dir;
          tadj.adjacent_tris_back_dir_[adj_dir] = dir;
        }
      }
    }
  }
  adjacent_tris_valid_ = true;
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
    int tadj_idx = t.adjacent_tris_[dir];
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

// make all windings point the same way
void mesh_core::Mesh::fixWindings()
{
  int mark = 0;

  findSubmeshes();
  clearMark(mark);


  for (int i=0; i<number_of_submeshes_; ++i)
  {
    int tri_cnt = 0;
    int flip_cnt = 0;
    fixWindings(
              tris_[submesh_tris_[i]],
              false,
              ++mark,
              tri_cnt,
              flip_cnt);
    if (flip_cnt > tri_cnt/2)
    {
      // guessed wrong - flip them all back the opposite way
      fixWindings(
              tris_[submesh_tris_[i]],
              true,
              ++mark,
              tri_cnt,
              flip_cnt);
    }
  }
}

bool mesh_core::Mesh::isWindingSame(Triangle& tri, int dir)
{
  int adj_idx = tri.adjacent_tris_[dir];
  if (adj_idx < 0)
    return false;
  Triangle& adj = tris_[adj_idx];

  int back_dir = tri.adjacent_tris_back_dir_[dir];

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
void mesh_core::Mesh::fixWindings(
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
      int adj_idx = tri.adjacent_tris_[dir];
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
        logError("fixWindings OVERFLOWED!");
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
  if (adjacent_tris_valid_)
  {
    std::swap(tri.adjacent_tris_[0], tri.adjacent_tris_[2]);
    std::swap(tri.adjacent_tris_back_dir_[0], tri.adjacent_tris_back_dir_[2]);


    if (tri.adjacent_tris_[0] >= 0)
    {
      ACORN_ASSERT_TRI_IDX(tri.adjacent_tris_[0]);
      Triangle& adj = tris_[tri.adjacent_tris_[0]];
      ACORN_ASSERT_TRI(adj);
      adj.adjacent_tris_back_dir_[tri.adjacent_tris_back_dir_[0]] = 0;
    }

    if (tri.adjacent_tris_[2] >= 0)
    {
      ACORN_ASSERT_TRI_IDX(tri.adjacent_tris_[2]);
      Triangle& adj = tris_[tri.adjacent_tris_[2]];
      ACORN_ASSERT_TRI(adj);
      adj.adjacent_tris_back_dir_[tri.adjacent_tris_back_dir_[2]] = 2;
    }
  }
  assertValidTri(tri, "flipWinding2");
}

void mesh_core::Mesh::assertValidTri(
      const Triangle& tri,
      const char *msg) const
{
  ACORN_ASSERT_TRI(tri);
  ACORN_ASSERT_TRI_IDX(triIndex(tri));

  ACORN_ASSERT(adjacent_tris_valid_);
  ACORN_ASSERT(number_of_submeshes_>0);
  ACORN_ASSERT(number_of_submeshes_ == submesh_tris_.size());

  ACORN_ASSERT(tri.submesh_ >= 0 && tri.submesh_ < number_of_submeshes_);

  for (int dir = 0 ; dir < 3 ; ++dir)
  {
    ACORN_ASSERT_VERT_IDX(tri.verts_[dir]);
    ACORN_ASSERT_EDGE_IDX(tri.edges_[dir]);
    const Edge& edge = edges_[tri.edges_[dir]];
    ACORN_ASSERT_VERT_IDX(edge.verts_[0]);
    ACORN_ASSERT_VERT_IDX(edge.verts_[1]);
    if (edge.tris_[0] == -2)
    {
      ACORN_ASSERT(edge.tris_[1] > 2);
      ACORN_ASSERT(tri.adjacent_tris_[dir] == -2);
      ACORN_ASSERT(have_degenerate_edges_);
    }
    else
    {
      ACORN_ASSERT_TRI_IDX(edge.tris_[0]);
      if (edge.tris_[1] == -1)
      {
        ACORN_ASSERT(tri.adjacent_tris_[dir] == -1);
      }
      else
      {
        ACORN_ASSERT_TRI_IDX(edge.tris_[1]);
        ACORN_ASSERT_TRI_IDX(tri.adjacent_tris_[dir]);
        const Triangle& adj = tris_[tri.adjacent_tris_[dir]];
        int back_dir = tri.adjacent_tris_back_dir_[dir];
        ACORN_ASSERT_DIR(back_dir);

        ACORN_ASSERT(adj.adjacent_tris_[back_dir] == triIndex(tri));
        ACORN_ASSERT(adj.edges_[back_dir] == tri.edges_[dir]);
        ACORN_ASSERT(adj.adjacent_tris_back_dir_[back_dir] == dir);
      }
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


#if XXXXXXXX
void mesh_core::Mesh::fillGaps()
{
  findSubmeshes();

  std::vector<Edge>::iterator edge = edges_.begin();
  std::vector<Edge>::iterator edge_end = edges_.end();
  for ( ; edge != edge_end ; ++edge)
  {
    if (edge->tris_[0] >=0 && edge->tris_[1] < 0)
    {
      fillGap(*edge);
    }
  }
}

void mesh_core::Mesh::fillGap(Edge& edge)
{
  std::vector<Edge*> edges;
  edges.reserve(tris_.size());

  edges.push_back(edge);

  if (edge->tris_[0] < 0 || edge->tris_[1] >= 0)
  {
    logError("PROGRAMMING ERROR at %s:%d",__FILE__,__LINE__);
    return;
  }
  Triangle *t = &tris_[edge->tris_[0]];
  for (int dir = 0 ; dir < 3 ; ++dir)
  {
    if (t->edges_
  }
}

void mesh_core::Mesh::fillGap(Edge* edge)
{
  std::vector<Edge*> edges;
  edges.reserve(tris_.size());

  edges.push_back(edge);

  if (edge->tris_[0] < 0 || edge->tris_[1] >= 0)
  {
    logError("PROGRAMMING ERROR at %s:%d",__FILE__,__LINE__);
    return;
  }
  Triangle *t = &tris_[edge->tris_[0]];
  for (int dir = 0 ; dir < 3 ; ++dir)
  {
    if (t->edges_
  }
}
#endif
