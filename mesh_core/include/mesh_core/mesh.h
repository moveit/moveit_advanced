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
#include <console_bridge/console.h>
#include <mesh_core/geom.h>

#define MESH_CORE__MESH__ENABLE_DEBUG 1

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



  
namespace mesh_core
{
class Plane;

class Mesh
{
public:
  // info about an triangle's edge
  struct TriEdge
  {
    // index of edge adjacent to triangle.  Always valid.
    int edge_idx_;

    // index in Edge::tris_ of this triangle.  Always valid.
    int edge_tri_idx_;

    // Only valid if adjacent_tris_valid_ is true.
    //   -1 : no adjacent triangle  (or unknown if !adjacent_tris_valid_)
    //   -2 : more than 1 adjacent triangle
    //   else : index of adjacent triangle
    int adjacent_tri_;

    // if adjacent_tri_ >= 0 then this is the dir in that adjacent triangle
    // corresponding to this triangle.  So
    //   int tria = <any-tri-idx>
    //   int dir = <0, 1, or 2>
    //   int trib = mesh.tris_[tria].edges_[dir].adjacent_tri_
    //   int dirb = mesh.tris_[tria].edges_[dir].adjacent_tri_back_dir_
    //   tria ==    mesh.tris_[trib].edges[dirb].adjacent_tri_
    int adjacent_tri_back_dir_;
  };
  struct Triangle
  {
    // true if same vertices in same winding order.
    bool operator==(const Triangle& b) const;

    // index of vertices (always valid)
    int verts_[3];

    // info about each edge of triangle.  Always valid.
    TriEdge edges_[3];

    // index of submesh
    // -1 until findSubmeshes() is called.
    int submesh_;

    // used in various places when we want to see all tris exactly once
    int mark_;
  };

  // info about an edge's triangle
  struct EdgeTri
  {
    // index of tri adjacent to edge.  Always valid.
    int tri_idx_;

    // dir in tri of this edge.  Always valid.
    int tri_dir_;
  };
  struct Edge
  {
    // index of vertices.
    // always valid.
    // verts_[0] < verts_[1] (guaranteed)
    int verts_[2];

    // info about adjacent tris.
    // In a well formed mesh there should be 2 tris adjacent to every edge.
    std::vector<EdgeTri> tris_;

    // hack to work around deficiencies of fillGap()
    // TODO: can this be fixed?
    bool hack_ignore_gap_;
  };

  struct Vertex
  {
    // Note: the position of the vertex is in the Mesh::verts_ array.

    // indices of edges touching this vertex.
    std::vector<int> edges_;

    // indices of triangles touching this vertex.
    std::vector<int> tris_;
  };

  // construct an empty mesh
  // vertices closer than epsilon will be combined.
  // degenerate triangles are removed as they are added.
  explicit Mesh(double epsilon=0.00001);

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
  // ntris - number of triangles to add
  // tris - each tri is represented by 3 vertex indices
  // verts - the vertices
  void add(int ntris,
           int *tris,
           const EigenSTL::vector_Vector3d& verts);

  /// add a list of triangles.
  // ntris - number of triangles to add
  // tris - each tri is represented by 3 vertex indices
  // verts - each vertex is 3 doubles: x,y,z
  void add(int ntris,
           int *tris,
           double *verts);

  // add a sphere to the mesh.
  // Usually used with a previously empty mesh.
  // max_error is the amount of error allowed.
  // Vertices will be on the sphere and faces will be slightly inside the
  // sphere (but no more than max_error from sphere surface).
  void addSphere(const Eigen::Vector3d& center,
                 double radius,
                 double max_error);
  
  // set a different epsilon.  Does not affect existing vertices.
  void setEpsilon(double epsilon);

  // access. Low level - no check on idx!
  const Eigen::Vector3d& getVert(int idx) const { return verts_[idx]; }
  const Triangle& getTri(int idx) const { return tris_[idx]; }
  int getTriCount() const { return tris_.size(); }
  int getVertCount() const { return verts_.size(); }

  const EigenSTL::vector_Vector3d& getVerts() const { return verts_; }
  const std::vector<Triangle>& getTris() const { return tris_; }
  void getTris(std::vector<int>& tris) const;

  // get tight bounding sphere around mesh
  void getBoundingSphere(Eigen::Vector3d& center,
                         double &radius) const;

  // get tight bounding sphere around mesh
  void getAABB(Eigen::Vector3d& min,
               Eigen::Vector3d& max) const;

  // get face normals (one for each tri, indexed by tri index)
  const EigenSTL::vector_Vector3d& getFaceNormals() const;

  struct SphereRepNode
  {
    SphereRepNode *parent_;
    SphereRepNode *first_child_;
    SphereRepNode *next_sibling_;
    SphereRepNode *prev_sibling_;
    const Mesh *mesh_;

    // same as mesh_ if this is a temporary mesh that should be deleted.
    // Otherwise NULL.
    Mesh *tmp_mesh_;

    // below is for debugging

    int depth_;   // number of parents

    Plane plane_; // plane used to split children

    // 1<<depth_ always set
    // 1<<level indicates which child of that level
    int id_;

    // points used to pick plane
    EigenSTL::vector_Vector3d plane_points_;

    // closest vertex to center of sphere
    Eigen::Vector3d closest_point;

    // farthest vertex from center axis
    Eigen::Vector3d farthest_point_;

    // closest triangle to center of sphere
    EigenSTL::vector_Vector3d closes_triangle_;
  };

  // generate a set of spheres that represents the mesh
  //   tolerance - how well the spheres must fit.
  //   sphere_centers - result
  //   sphere_radii   - result
  //   mesh_tree - optional - returns sub meshes for debugging.
  //   max_depth - recurse at most this deep when splitting (-1 = no limit)
  void getSphereRep(
          double tolerance,
          EigenSTL::vector_Vector3d& sphere_centers,
          std::vector <double> sphere_radii,
          SphereRepNode **mesh_tree = NULL,
          int max_depth = -1) const;

  // delete a mesh_tree returned by getSphereRep()
  static void deleteSphereRepTree(SphereRepNode *mesh_tree);

  // harvest spheres from leaves of tree.
  // Nodes at max_depth are treated as leaves.
  // Using max_depth = 0 will return just spheres from mesh_tree and its
  // siblings (if any).
  // Using max_depth = -1 (default) will return all spheres.
  static void collectSphereRepSpheres(
                      SphereRepNode *mesh_tree,
                      EigenSTL::vector_Vector3d& sphere_centers,
                      std::vector<double>& sphere_radii,
                      int max_depth = -1);


  // slize this mesh in half along plane and create 2 new meshes
  //   a: contains only parts beyond plane
  //   b: contains only parts before plane
  // The original mesh is not modified
  void slice(const Plane& plane,
             Mesh& a,
             Mesh& b) const;


  // make windings agree on adjacent triangles, so adjacent tris have
  // consistent normals.  Attempts to make all windings CCW when looking from
  // outside the mesh to inside.
  void fixWindings();

  // fill in gaps (cracks or holes) in the mesh
  void fillGaps();

  // find point on surface of mesh closest to given point.
  // return closest_point which is on mesh surface.
  // return index of triangle 
  // return distance from point to closest_point.
  //
  // Mesh must not be empty.  If it is -1.0 is returned.
  double findClosestPoint(const Eigen::Vector3d& point,
                          Eigen::Vector3d& closest_point,
                          int& closest_tri) const;
public:
  //#########################################################################
  //############################### DEBUG ###################################
  //#########################################################################

  // print index info
  void print() const;

  // Info about a gap for debugging
  struct GapDebugInfo
  {
    // vertices of gap loop
    EigenSTL::vector_Vector3d points_;

    // vertex indices of gap loop
    std::vector<int> verts_;

    // tris used to fill gap
    std::vector<int> gap_tris_;
    
    // tris neighboring gap
    std::vector<int> neigbor_tris_;
  };

  // get gap debug info
  const std::vector<GapDebugInfo>& getGapDebugInfo() const
  {
    return gap_debug_;
  }

  // enable debugging (debugGapInfo())
  static void enableDebugging(bool enable);

private:
  int triIndex(const Triangle& tri) const
  {
    return &tri - &tris_[0];
  }
  int edgeIndex(const Edge& edge) const
  {
    return &edge - &edges_[0];
  }
  static int nextDir(int dir)
  {
    return (dir + 1) % 3;
  }
  static int prevDir(int dir)
  {
    return (dir + 2) % 3;
  }
  static int otherVertIndex(const Edge& edge, int this_vert_index)
  {
    ACORN_ASSERT(this_vert_index == edge.verts_[0] ||
                 this_vert_index == edge.verts_[1]);
    return this_vert_index == edge.verts_[0] ?
                              edge.verts_[1] :
                              edge.verts_[0];
  }


  /// add a triangle defined by 3 existing vertices. 
  void add(int a,
           int b,
           int c);

  // find/add a vertex and return its (possibly new) index.
  int addVertex(const Eigen::Vector3d& a);

  // find/add an edge given index of 2 vertices
  int addEdge(int vertidx_a, int vertidx_b);

  // used by addSphere()
  void addSphereTri(const Eigen::Vector3d& center,
                    double radius,
                    double max_error,
                    const Eigen::Vector3d& a,
                    const Eigen::Vector3d& b,
                    const Eigen::Vector3d& c);


  // set the adjacent_tris_ and adjacent_tris_back_dir_ fields in all triangles.
  void setAdjacentTriangles();
  
  // assign each triangle to a submesh.
  // Each submesh is connected to all other triangles in the same submesh, but
  // not connected to triangles in other submeshes.
  void findSubmeshes();

  // helper for findSubmeshes().
  void assignSubmesh(Triangle& t, int submesh);

  // helper for fixWindings()()
  void fixSubmeshWindings(
      Triangle& tri,
      bool flip_first_tri,
      int mark,
      int& tri_cnt,
      int& flip_cnt);
  void flipWinding(Triangle& tri);

  // true if adjacent triangle winding is consistent with this triangle.
  // false if they differ or there are more/less than 1 adjacent tri in that dir
  bool isWindingSame(Triangle& tri, int dir);

  // true if winding for this mesh appears to be CCW.
  // Only guaranteed to work for gapless nonintersecting submeshes
  bool isWindingCCW(int submesh) const;

  // set the mark_ field in all triangles to the given value
  void clearMark(int value = 0);

  // find a gap and return an edge touching the gap.  NULL if no gaps.
  // Used by fillGaps()
  Edge* findGap();

  // fill a gap touching this edge. Used by fillGaps()
  void fillGap(Edge& first_edge);

  // triangulate a polygon described by verts.
  // Triiangles are added to mesh.
  // Winding order of triangles is reverse of vert order.
  // verts contains verts_ indices.
  //
  // if partial_ok is true then generatePolygon may only triangulate part of
  // thepolygon (but always at least 1 triangle).  If false the entire polygon
  // will be triangulated.
  void generatePolygon(const std::vector<int> verts, bool partial_ok);

  struct GapPoint;

  // add a gap-filling triangle (used by fillGaps())
  void addGapTri(const GapPoint *p, double direction);

  // check whether point is an ear.  Used by fillGap().
  static void calcEarState(
                      GapPoint& point,
                      const std::vector<GapPoint>& points);

  struct GetSphereRepParams;

  // recursive helper for getSphereRep()
  void calculateSphereRep(
          const GetSphereRepParams& params,
          SphereRepNode *mesh_tree) const;

  // create 2 children for mesh_node (used by getSphereRep())
  bool calculateSphereRepTreeSplit(
          double tolerance,
          SphereRepNode *mesh_node) const;

  // split mesh in 2 for getSphereRep()
  bool calculateSphereRepMeshSplit(
          double tolerance,
          SphereRepNode *mesh_node,
          Mesh **mesh_a,
          Mesh **mesh_b) const;

  // calculate splitting plane for getSphereRep()
  bool calculateSphereRepSplitPlane(
          double tolerance,
          SphereRepNode *mesh_node,
          Plane& plane) const;

  // implement calculateSphereRepSplitPlane() by finding plane perpendular to
  // liner between bounding sphere center and farthest vertex from bounding
  // sphere center.
  bool calculateSphereRepSplitPlane_far(
          double tolerance,
          SphereRepNode *mesh_node,
          Plane& plane) const;

  // implement calculateSphereRepSplitPlane() by
  //  1) find closest point on original mesh to center of bounding sphere
  //  2) y_axis to line through that point and bsphere center
  //  3) find farthest vertex on submesh from y_axis
  //  4) z_axis is from y_axis to farthest point
  //  5) plane contains y_axis and perpendicular to z_axis
  bool calculateSphereRepSplitPlane_closeFar(
          double tolerance,
          SphereRepNode *mesh_node,
          Plane& plane) const;

  // implement calculateSphereRepSplitPlane() by
  //   1) if parents, splitting plane kis perpendicular to parent(s)
  //   2) otherwise split largest axis-aligned dimension
  bool calculateSphereRepSplitPlane_ortho(
          double tolerance,
          SphereRepNode *mesh_node,
          Plane& plane) const;

  // implement calculateSphereRepSplitPlane() by splitting along largest of
  // x,y,z axes
  bool calculateSphereRepSplitPlane_bigAxis(
          double tolerance,
          SphereRepNode *mesh_node,
          Plane& plane) const;


  // Debug asserts
  void assertValidTri(const Triangle& tri, const char *msg) const;
  void assertValidTri_PreAdjacentValid(const Triangle& tri, const char *msg) const;
  void assertValidEdge(const Edge& edge, const char *msg) const;


private:
  double epsilon_;
  double epsilon_squared_;
  EigenSTL::vector_Vector3d verts_;
  std::vector<Triangle> tris_;
  std::vector<Edge> edges_;

  // map from vertex idx pair to edge idx.
  // pair<a,b> always has a<b
  std::map<std::pair<int,int>,int> edge_map_;

  // extra info per vertex. 
  // Note that the position of the vertex is in verts_
  std::vector<Vertex> vert_info_;

  // true if any edges are shared by >2 triangles
  bool have_degenerate_edges_;

  // calculated by findSubmeshes().  Invalid if -1
  int number_of_submeshes_;

  // one triangle from each submesh.
  std::vector<int> submesh_tris_;

  // set true by setAdjacentTriangles()
  bool adjacent_tris_valid_;

  // bounding sphere
  mutable bool bounding_sphere_valid_;
  mutable Eigen::Vector3d bounding_sphere_center_;
  mutable double bounding_sphere_radius_;

  // aabb
  mutable bool aabb_valid_;
  mutable Eigen::Vector3d aabb_min_;
  mutable Eigen::Vector3d aabb_max_;

  // face normals
  mutable EigenSTL::vector_Vector3d face_normals_;

  // enable debugging features
  static bool debug_;

  // info about filled gaps
  std::vector<GapDebugInfo> gap_debug_;
};

}

inline void mesh_core::Mesh::enableDebugging(bool enable)
{
  debug_ = enable;
}


#if !MESH_CORE__MESH__ENABLE_DEBUG
inline void mesh_core::Mesh::assertValidTri(
      const Triangle& tri, 
      const char *msg) const
{}

inline void mesh_core::Mesh::assertValidTri_PreAdjacentValid(
      const Triangle& tri, 
      const char *msg) const
{}

inline void mesh_core::Mesh::assertValidEdge(
      const Edge& edge,
      const char *msg) const
{}
#endif

#endif

