#ifndef STUDENT_CODE_H
#define STUDENT_CODE_H

#include "halfEdgeMesh.h"
#include "bezierPatch.h"
#include "bezierCurve.h"

using namespace std;

namespace CGL {

  class MeshResampler{

  public:

    MeshResampler(){};
    ~MeshResampler(){}

    void upsample(HalfedgeMesh& mesh);
    void ball_pivot(HalfedgeMesh& mesh);
    VertexIter calculateBallPointDemo( Halfedge h, HalfedgeMesh& mesh);
  };
}

#endif // STUDENT_CODE_H
