#ifndef STUDENT_CODE_H
#define STUDENT_CODE_H

#include "halfEdgeMesh.h"
#include "bezierPatch.h"
#include "bezierCurve.h"
#include <unordered_map>

using namespace std;

namespace CGL {

  class MeshResampler{

  public:

    MeshResampler(){};
    ~MeshResampler(){}

    void upsample(HalfedgeMesh& mesh);
    std::vector<VertexIter> ball_pivot(HalfedgeMesh& mesh, HalfedgeIter& populate);
    bool calculateBallPointDemo( Halfedge h, HalfedgeMesh& mesh, VertexIter& populate);
    double set_rho(HalfedgeMesh& mesh, double rho);
    std::vector<VertexIter > get_neighbors(Vector3D p);
    unordered_map<int, vector<VertexIter > *> cluster_vertices (HalfedgeMesh& mesh);

    unordered_map<int, vector<VertexIter > *> map;
    double x_min;
    double y_min;
    double z_min;
    double x_max;
    double y_max;
    double z_max;
    double mod;
    double rho;
  };
}

#endif // STUDENT_CODE_H
