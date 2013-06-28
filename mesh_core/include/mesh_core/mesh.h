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

#define MESH_CORE__MESH__ENABLE_DEBUG 1

namespace mesh_core
{

class Mesh
{
public:
  struct Triangle
  {
    // true if same vertices in same winding order.
    bool operator==(const Triangle& b) const;

    // index of vertices (always valid)
    int verts_[3];

    // index of edges (always valid)
    // edges_[0] is verts_[0] - verts_[1]
    // edges_[1] is verts_[1] - verts_[2]
    // edges_[2] is verts_[2] - verts_[0]
    int edges_[3];

    // index of adjacent triangle sharing edge edges_[i]
    // -1 if none, -2 if more than 1
    // invalid unless adjacent_tris_valid_ is true
    int adjacent_tris_[3];

    // if adjacent_tris_[dir] is valid then this is the dir in the adjacent
    // triangle corresponding to this triangle.  In other words:
    //   int tria = <any-tri-idx>
    //   int dir = <0, 1, or 2>
    //   int trib = mesh.tris_[tria].adjacent_tris_[dir]
    //   int dirb = mesh.tris_[tria].adjacent_tris_back_dir_[dir]
    //   tria ==    mesh.tris_[trib].adjacent_tris_[dirb]
    int adjacent_tris_back_dir_[3];

    // index of submesh
    // -1 until findSubmeshes() is called.
    int submesh_;

    // used in various places when we want to see all tris exactly once
    int mark_;
  };
  struct Edge
  {
    // index of vertices.
    // always valid.
    // verts_[0] < verts_[1] (guaranteed)
    int verts_[2];

    // index of adjacent tris in same submesh.
    // if >2 tris then tris_[0]=-2 and tris_[1] = # of tris
    int tris_[2];
  };

  // construct an empty mesh
  // vertices closer than epsilon will be combined.
  // degenerate triangles are removed as they are added.
  Mesh(double epsilon=0.001);

  /// remove everything and be left with empty mesh
  void clear();

  /// reserve space for tris and/or verts in addition to what is already stored.
  // This is an optimization and is completely optional.
  void reserve(int ntris, int nverts = 0);

  /// add a triangle defined by 3 vertices. 
  void add(const Eigen::Vector3d& a,
           const Eigen::Vector3d& b,
           const Eigen::Vector3d& c);

  /// add a triangle defined by 3 vertices. 
  // Each vertex is a pointer to 3 // doubles.
  void add(double *a,
           double *b,
           double *c);

  /// add a list of triangles.
  // tris is a pointer to ntris*3 ints.  Each triangle is represented by 3
  // adjacent ints.  The ints are indices into the verts array.
  void add(const EigenSTL::vector_Vector3d& verts,
           int ntris,
           int *tris);

  /// add a list of triangles.
  // tris is a pointer to ntris*3 ints.  Each triangle is represented by 3
  // adjacent ints.  The ints are vertex indices.
  // verts contains 3 doubles per vertex: x, y, z
  void add(double *verts,
           int ntris,
           int *tris);
  

  // set a different epsilon.  Does not affect existing vertices.
  void setEpsilon(double epsilon);

  // access. Low level - no check on idx!
  const Eigen::Vector3d& getVert(int idx) const { return verts_[idx]; }
  const Triangle& getTri(int idx) const { return tris_[idx]; }
  int getTriCount() const { return tris_.size(); }
  int getVertCount() const { return verts_.size(); }

  const EigenSTL::vector_Vector3d& getVerts() const { return verts_; }
  const std::vector<Triangle>& getTris() const { return tris_; }

  // make windings agree on adjacent triangles, so adjacent tris have
  // consistent normals.
  void fixWindings();

  // fill in gaps (cracks or holes) in the mesh
  void fillGaps();

  int triIndex(const Triangle& tri) const
  {
    return &tri - &tris_[0];
  }
  int edgeIndex(const Edge& edge) const
  {
    return &edge - &edges_[0];
  }

private:
  // find/add a vertex and return its (possibly new) index.
  int addVertex(const Eigen::Vector3d& a);

  // find/add an edge given index of 2 vertices
  int addEdge(int vertidx_a, int vertidx_b);

  // set the adjacent_tris_ and adjacent_tris_back_dir_ fields in all triangles.
  void setAdjacentTriangles();
  
  // assign each triangle to a submesh.
  // Each submesh is connected to all other triangles in the same submesh, but
  // not connected to triangles in other submeshes.
  void findSubmeshes();

  // helper for findSubmeshes().
  void assignSubmesh(Triangle& t, int submesh);

  // helper for fixWindings()()
  void fixWindings(
      Triangle& tri,
      bool flip_first_tri,
      int mark,
      int& tri_cnt,
      int& flip_cnt);
  void flipWinding(Triangle& tri);

  // true if adjacent triangle winding is consistent with this triangle.
  // false if they differ or there are more/less than 1 adjacent tri in that dir
  bool isWindingSame(Triangle& tri, int dir);

  // set the mark_ field in all triangles to the given value
  void clearMark(int value = 0);

  double epsilon_;
  double epsilon_squared_;
  EigenSTL::vector_Vector3d verts_;
  std::vector<Triangle> tris_;
  std::vector<Edge> edges_;

  // map from vertex idx pair to edge idx.
  // pair<a,b> always has a<b
  std::map<std::pair<int,int>,int> edge_map_;

  // true if any edges are shared by >2 triangles
  bool have_degenerate_edges_;

  // calculated by findSubmeshes().  Invalid if -1
  int number_of_submeshes_;

  // one triangle from each submesh.
  std::vector<int> submesh_tris_;

  // set true by setAdjacentTriangles()
  bool adjacent_tris_valid_;



  void assertValidTri(const Triangle& tri, const char *msg) const
#if MESH_CORE__MESH__ENABLE_DEBUG
  ;
#else
  {}
#endif
};

}

#if MESH_CORE__MESH__ENABLE_DEBUG
#define ACORN_ASSERT_LINE(_cond, _file, _line) \
  do { \
    if (!(_cond)) { \
      logInform("Failed assert at %s:%d  -- %s", \
        _file, \
        _line, \
        #_cond); \
      abort(); \
    } \
  } while(0)
#else
#define ACORN_ASSERT_LINE(_cond, _file, _line) \
  do { } while(0)
#endif

#define ACORN_ASSERT(_cond) \
        ACORN_ASSERT_LINE(_cond, __FILE__, __LINE__)

#define ACORN_ASSERT_DIR(d) \
  ACORN_ASSERT((unsigned int)(d) < 3)
#define ACORN_ASSERT_TRI_IDX(t) \
  ACORN_ASSERT((unsigned int)(t) < this->tris_.size())
#define ACORN_ASSERT_VERT_IDX(v) \
  ACORN_ASSERT((unsigned int)(v) < this->verts_.size())
#define ACORN_ASSERT_EDGE_IDX(e) \
  ACORN_ASSERT((unsigned int)(e) < this->edges_.size())
#define ACORN_ASSERT_TRI(t) \
  ACORN_ASSERT_TRI_IDX(&tri - &this->tris_[0])
  

#endif

