#include "student_code.h"
#include "mutablePriorityQueue.h"
#include <deque>
#include <algorithm>
#include <math.h>

#define PI 3.14159265

using namespace std;

namespace CGL
{
  void BezierCurve::evaluateStep()
  {
    // TODO Part 1.
    // Perform one step of the Bezier curve's evaluation at t using de Casteljau's algorithm for subdivision.
    // Store all of the intermediate control points into the 2D vector evaluatedLevels.

    if (evaluatedLevels[evaluatedLevels.size() - 1].size() > 1) {

      evaluatedLevels.push_back( std::vector<Vector2D>() );
      Vector2D a, b;
      for ( int i = 0; i < evaluatedLevels[evaluatedLevels.size() - 2].size() - 1; i++) {
        a = evaluatedLevels[evaluatedLevels.size() - 2][i];
        b = evaluatedLevels[evaluatedLevels.size() - 2][i + 1];

        evaluatedLevels[ evaluatedLevels.size() - 1 ].push_back((1.0 -t)*a + t*b);
      }

    }

  }


  Vector3D BezierPatch::evaluate(double u, double v) const
  {
    // TODO Part 2.
    // Evaluate the Bezier surface at parameters (u, v) through 2D de Casteljau subdivision.
    // (i.e. Unlike Part 1 where we performed one subdivision level per call to evaluateStep, this function
    // should apply de Casteljau's algorithm until it computes the final, evaluated point on the surface)
    std::vector<Vector3D> currentLevels = std::vector<Vector3D>();
    for (int i = 0; i < controlPoints.size(); i ++) {
      currentLevels.push_back(BezierPatch::evaluate1D(controlPoints[i], u));
    }

    return BezierPatch::evaluate1D(currentLevels, v);
  }

  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> points, double t) const
  {
    // TODO Part 2.
    // Optional helper function that you might find useful to implement as an abstraction when implementing BezierPatch::evaluate.
    // Given an array of 4 points that lie on a single curve, evaluates the Bezier curve at parameter t using 1D de Casteljau subdivision.
    std::vector<Vector3D> currentLevel = points;
    //currentLevels.push_back(points);
    Vector3D a, b;
    std::vector<Vector3D> newLevel;
    while (currentLevel.size() > 1) {
      newLevel = std::vector<Vector3D>();
      for ( int i = 0; i < currentLevel.size() - 1; i++) {
        a = currentLevel[i];
        b = currentLevel[i + 1];
        newLevel.push_back((1.0 -t)*a + t*b);
      }
      currentLevel = newLevel;

    }
    return  currentLevel[0];
  }



  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // TODO Returns an approximate unit normal at this vertex, computed by
    // TODO taking the area-weighted average of the normals of neighboring
    // TODO triangles, then normalizing.
    Vector3D n(0.,0.,0.); // initialize a vector to store your normal sum
    HalfedgeCIter h = halfedge(); // Since we're in a Vertex, this returns a halfedge
                                 // pointing _away_ from that vertex
    do {

      n += cross(  h->vertex()->position - h->twin()->vertex()->position,  h->next()->vertex()->position - h->next()->twin()->vertex()->position);
      h = h->twin()->next();

    } while (h != _halfedge);


    return n.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // TODO This method should flip the given edge and return an iterator to the flipped edge.
    if (e0->isBoundary() ) {
      printf("Cannot flip a boundary edge.\n" );
      return e0;
    }

    HalfedgeIter h = e0->halfedge();
    HalfedgeIter tmp = h->next();
    HalfedgeIter twinTmp = h->twin()->next();
    HalfedgeIter tmp2 = tmp->next();
    HalfedgeIter twinTmp2 = twinTmp->next();
    FaceIter topFace = h->twin()->face();
    FaceIter bottomFace = h->face();
    VertexIter topV = h->twin()->vertex();
    VertexIter rightV = tmp2->vertex();
    VertexIter bottomV = h->vertex();
    VertexIter leftV = twinTmp2->vertex();
    EdgeIter e1 = twinTmp->edge();
    EdgeIter e2 = twinTmp2->edge();
    EdgeIter e3 = tmp->edge();
    EdgeIter e4 = tmp2->edge();


    h->setNeighbors         ( tmp2,
                                h->twin(),
                                leftV,
                                h->edge(),
                                bottomFace );

    tmp2->setNeighbors      ( twinTmp,
                                  tmp2->twin(),
                                  rightV,
                                  tmp2->edge(),
                                  bottomFace );

    twinTmp->setNeighbors   ( h,
                                  twinTmp->twin(),
                                  bottomV,
                                  twinTmp->edge(),
                                  bottomFace );

    twinTmp2->setNeighbors  ( tmp,
                                  twinTmp2->twin(),
                                  leftV,
                                  twinTmp2->edge(),
                                  topFace);

    tmp->setNeighbors       ( h->twin(),
                                  tmp->twin(),
                                  topV,
                                  tmp->edge(),
                                  topFace );

    h->twin()->setNeighbors ( twinTmp2,
                                h,
                                rightV,
                                h->twin()->edge(),
                                topFace );

    e0->halfedge()=h;
    e1->halfedge()=twinTmp;
    e2->halfedge()=twinTmp2;
    e3->halfedge()=tmp;
    e4->halfedge()=tmp2;

    topV->halfedge() = tmp;
    bottomV->halfedge() = twinTmp;
    rightV ->halfedge() = tmp2;
    leftV->halfedge() = twinTmp2;

    topFace->halfedge() = tmp;
    bottomFace->halfedge()=twinTmp;


    return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // TODO This method should split the given edge and return an iterator to the newly inserted vertex.
    // TODO The halfedge of this vertex should point along the edge that was split, rather than the new edges.

    if (e0->halfedge()->face()->isBoundary()) {
      /*Edge is pointing to the boundary halfedge*/
      HalfedgeIter h = e0->halfedge()->twin();
      HalfedgeIter boundary_halfedge = e0->halfedge();
      HalfedgeIter tmp2 = h->next();
      HalfedgeIter tmp = tmp2->next();

      VertexIter topV = tmp->vertex();
      VertexIter rightV = tmp2->vertex();
      VertexIter leftV = h->vertex();
      VertexIter middleV = this->newVertex();
      middleV->isNew = true;

      EdgeIter e1 = this->newEdge();
      EdgeIter e2 = this->newEdge();
      e1->isNew = true;
      e2->isNew = false;

      FaceIter topLeftFace = h->face();
      FaceIter topRightFace = this->newFace();
      FaceIter boundaryFace = this->newBoundary();

      HalfedgeIter ul = this->newHalfedge();
      HalfedgeIter ur = this->newHalfedge();
      HalfedgeIter rt = this->newHalfedge();
      HalfedgeIter ru = this->newHalfedge();

      topLeftFace->halfedge() = ul;
      topRightFace->halfedge() = ur;
      boundaryFace->halfedge() = ru;

      e1->halfedge() = ur;
      e2->halfedge() = rt;
      e0->halfedge() = h;

      middleV->halfedge() = h;
      rightV->halfedge() = rt;
      leftV->halfedge() = h;
      topV->halfedge() = ul;
      middleV->position = h->vertex()->position*0.5 + tmp2->vertex()->position*0.5;

      /*Top left quadrant */
      h->setNeighbors         ( ul,
                                  ru,
                                  leftV,
                                  e0,
                                  topLeftFace);

      ul->setNeighbors        ( tmp,
                                ur,
                                middleV,
                                e1,
                                topLeftFace );

      tmp->setNeighbors       ( h,
                                tmp->twin(),
                                topV,
                                tmp->edge(),
                                topLeftFace );

      //topRight quadrant
      rt->setNeighbors        ( tmp2,
                                boundary_halfedge,
                                middleV,
                                e2,
                                topRightFace);

      tmp2->setNeighbors      ( ur,
                                tmp2->twin(),
                                rightV,
                                tmp2->edge(),
                                topRightFace );

      ur->setNeighbors        ( rt,
                                ul,
                                topV,
                                e1,
                                topRightFace );

      //Boundary face
      ru->setNeighbors        ( boundary_halfedge->next(),
                                h,
                                middleV,
                                e0,
                                boundaryFace);

      boundary_halfedge->setNeighbors ( ru,
                            rt,
                            rightV,
                            e2,
                            boundary_halfedge->face() );
      return middleV;

    } else if (e0->halfedge()->twin()->face()->isBoundary()) {
      /*Edge is pointing to the non boundary halfedge*/
      HalfedgeIter h = e0->halfedge();
      HalfedgeIter boundary_halfedge = e0->halfedge()->twin();
      HalfedgeIter tmp2 = h->next();
      HalfedgeIter tmp = tmp2->next();

      VertexIter topV = tmp->vertex();
      VertexIter rightV = tmp2->vertex();
      VertexIter leftV = h->vertex();
      VertexIter middleV = this->newVertex();
      middleV->isNew = true;

      EdgeIter e1 = this->newEdge();
      EdgeIter e2 = this->newEdge();
      e1->isNew = true;
      e2->isNew = false;

      FaceIter topLeftFace = h->face();
      FaceIter topRightFace = this->newFace();
      FaceIter boundaryFace = this->newBoundary();


      HalfedgeIter ul = this->newHalfedge();
      HalfedgeIter ur = this->newHalfedge();
      HalfedgeIter rt = this->newHalfedge();
      HalfedgeIter ru = this->newHalfedge();

      topLeftFace->halfedge() = ul;
      topRightFace->halfedge() = ur;
      boundaryFace->halfedge() = ru;


      e1->halfedge() = ur;
      e2->halfedge() = rt;
      e0->halfedge() = h;

      middleV->halfedge() = h;
      rightV->halfedge() = rt;
      leftV->halfedge() = h;
      topV->halfedge() = ul;
      middleV->position = h->vertex()->position*0.5 + tmp2->vertex()->position*0.5;


      /*Top left quadrant */
      h->setNeighbors         ( ul,
                                  ru,
                                  leftV,
                                  e0,
                                  topLeftFace);

      ul->setNeighbors        ( tmp,
                                ur,
                                middleV,
                                e1,
                                topLeftFace );

      tmp->setNeighbors       ( h,
                                tmp->twin(),
                                topV,
                                tmp->edge(),
                                topLeftFace );

      //topRight quadrant
      rt->setNeighbors        ( tmp2,
                                boundary_halfedge,
                                middleV,
                                e2,
                                topRightFace);

      tmp2->setNeighbors      ( ur,
                                tmp2->twin(),
                                rightV,
                                tmp2->edge(),
                                topRightFace );

      ur->setNeighbors        ( rt,
                                ul,
                                topV,
                                e1,
                                topRightFace );

      //Boundary face
      ru->setNeighbors        ( boundary_halfedge->next(),
                                h,
                                middleV,
                                e0,
                                boundaryFace);

      boundary_halfedge->setNeighbors ( ru,
                            rt,
                            rightV,
                            e2,
                            boundary_halfedge->face() );
      return middleV;

    } else {

      HalfedgeIter h = e0->halfedge();
      HalfedgeIter tmp = h->next();
      HalfedgeIter twinTmp = h->twin()->next();
      HalfedgeIter tmp2 = tmp->next();
      HalfedgeIter twinTmp2 = twinTmp->next();

      VertexIter topV = h->twin()->vertex();
      VertexIter rightV = tmp2->vertex();
      VertexIter bottomV = h->vertex();
      VertexIter leftV = twinTmp2->vertex();
      VertexIter middleV = this->newVertex();
      middleV->isNew = true;

      EdgeIter e1 = this->newEdge();
      EdgeIter e2 = this->newEdge();
      e1->isNew = true;
      e2->isNew = true;

      EdgeIter e3 = this->newEdge();
      e3->isNew = false;

      FaceIter topRightFace = h->face();
      FaceIter topLeftFace = h->twin()->face();

      FaceIter bottomRightFace = this->newFace();
      FaceIter bottomLeftFace = this->newFace();

      HalfedgeIter rt = this->newHalfedge();
      HalfedgeIter lt = this->newHalfedge();
      HalfedgeIter ru = this->newHalfedge();
      HalfedgeIter lu = this->newHalfedge();
      HalfedgeIter ur = this->newHalfedge();
      HalfedgeIter ul = this->newHalfedge();


      topLeftFace->halfedge() = lt;
      topRightFace->halfedge() = h;
      bottomRightFace->halfedge() = ru;
      bottomLeftFace->halfedge() = ul;

      e1->halfedge() = lt;
      e2->halfedge() = rt;
      e3->halfedge() = ul;
      e0->halfedge() = h;

      middleV->halfedge() = h;
      rightV->halfedge() = rt;
      bottomV->halfedge() = ur;
      leftV->halfedge() = twinTmp2;
      topV->halfedge() = tmp;

      middleV->position = h->vertex()->position*0.5 + h->next()->vertex()->position*0.5;


      //top Right quadrant
      h->setNeighbors         ( tmp,
                                  h->twin(),
                                  middleV,
                                  e0,
                                  topRightFace);

      tmp->setNeighbors       ( rt,
                                tmp->twin(),
                                topV,
                                tmp->edge(),
                                topRightFace );

      rt->setNeighbors        ( h,
                                ru,
                                rightV,
                                e2,
                                topRightFace );

      //topLeft quadrant
      h->twin()->setNeighbors ( lt,
                                h,
                                topV,
                                e0,
                                topLeftFace);

      lt->setNeighbors        ( twinTmp2,
                                lu,
                                middleV,
                                e1,
                                topLeftFace );

      twinTmp2->setNeighbors  ( h -> twin(),
                                twinTmp2->twin(),
                                leftV,
                                twinTmp2->edge(),
                                topLeftFace );


      //bottom Right quadrant
      ru->setNeighbors        (tmp2,
                                  rt,
                                  middleV,
                                  e2,
                                  bottomRightFace);

      tmp2->setNeighbors      (ur,
                                tmp2->twin(),
                                rightV,
                                tmp2->edge(),
                                bottomRightFace );

      ur->setNeighbors        (ru,
                                ul,
                                bottomV,
                                e3,
                                bottomRightFace );

      //bottom Left quadrant
      ul->setNeighbors        (twinTmp,
                                ur,
                                middleV,
                                e3,
                                bottomLeftFace );

      twinTmp->setNeighbors   (lu,
                                twinTmp->twin(),
                                bottomV,
                                twinTmp->edge(),
                                bottomLeftFace );

      lu->setNeighbors        (ul,
                                lt,
                                leftV,
                                e1,
                                bottomLeftFace );


      //printf("%f, %f, %f: \n" , middleV->position[0], middleV->position[1], middleV->position[2]);
      return middleV;
    }
  }


  double MeshResampler::set_rho(HalfedgeMesh& mesh, double rho) {
    printf("Setting rho\n");
    for (const auto &entry : map) {
      delete(entry.second);
    }
    printf("Deleted entries\n");
    map.clear();


    printf("Iterating...\n");
    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
      Vector3D pos = v->position;
      if (v == mesh.verticesBegin()) {
        x_min = pos.x;
        y_min = pos.y;
        z_min = pos.z;
        x_max = pos.x;
        y_max = pos.y;
        z_max = pos.z;
      } else {
        if (pos.x < x_min) {x_min = pos.x;}
        if (pos.y < y_min) {y_min = pos.y;}
        if (pos.z < z_min) {z_min = pos.z;}
        if (pos.x > x_max) {x_max = pos.x;}
        if (pos.y < y_max) {y_max = pos.y;}
        if (pos.z < z_max) {z_max = pos.z;}
      }
    }
    printf("Done iterating\n");
    double x_diff = x_max - x_min;
    double y_diff =  y_max - y_min;
    double z_diff =  z_max - z_min;

    printf("Doing some if cases\n");
    if (x_diff > y_diff) {
      mod = x_diff;
    } else {
      mod = y_diff;
    }
    if (z_diff > mod) {
      mod = z_diff;
    }
    printf("Those are done\n");
    mod = mod/(2.0* rho);
    int count = 0;
    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
      count++;
      Vector3D p = v->position;
      int w = (int) (p.x / mod);
      int h = (int) (p.y / mod);
      int t = (int) (p.z / mod);
      int hash = ((int) mod) * ((int) mod) * w + ((int) mod) * h + t;
      if (map.find(hash) == map.end()) {
        std::vector<VertexIter > * vec = new std::vector<VertexIter >();
        map[hash] = vec;
      }
      map[hash]->push_back(v);
    }
    return rho;
  }

  std::vector<VertexIter> MeshResampler::get_neighbors(Vector3D p) {
    printf("getting neighbors\n");
    int w = (int) (p.x / mod);
    int h = (int) (p.y / mod);
    int t = (int) (p.z / mod);
    int hash = ((int) mod) * ((int) mod) * w + ((int) mod) * h + t;
    std::vector<VertexIter>  * vec = new std::vector<VertexIter >();
    for (int i = -((int) mod)*((int) mod); i<= ((int) mod)*((int) mod); i += ((int) mod)*((int) mod)) {
      for (int j = -((int) mod); j <= ((int) mod); j += ((int) mod)) {
        for (int k = -1; k <= 1; k++) {
          int next_hash = hash + i + j + k;
          if (map.find(next_hash) != map.end()) {
            for (VertexIter ver : *map[next_hash]) {
              vec->push_back(ver);
              // printf("%4f %4f %4f\n", (ver)->position.x, (ver)->position.y, (ver)->position.z);
              // printf("%d\n", next_hash);
            }
          }
        }
      }
    }
    printf("done\n");
    printf("Sorting neighbors\n");
    // std::sort(vec->begin(), vec->end(), [&p] (const VertexIter lhs, const VertexIter rhs ){  return (lhs->position - p).norm() < (rhs->position - p).norm();});
    // std::sort( (*vec).begin( ), (*vec).end( ), []( const VertexIter& lhs, const VertexIter& rhs )
    // {
    //   return (lhs->position - v->position).norm() < (rhs->position - v->position).norm();
    // });
    printf("DOne\n");
    return *vec;
  }

  Vector3D circumcenter(Vector3D a, Vector3D b, Vector3D c) {

    Vector3D ac = c - a ;
    Vector3D ab = b - a ;
    Vector3D abXac = cross(ab, ac ) ;

    // this is the vector from a TO the circumsphere center
    return a + (cross(abXac, ab )*ac.norm2() + cross(ac, abXac )*ab.norm2()) / (2.0*abXac.norm2()) ;
  }

  bool circumsphere_center(Vector3D i , Vector3D j, Vector3D o, double rho, Vector3D& center) {
    Vector3D c = circumcenter(i, j, o);
    double alpha_sqr = (c - i).norm2();
    double discriminant = (rho*rho) - alpha_sqr;
    if (discriminant < 0.0) {
      return false;
    }
    double x_mag = sqrt(discriminant);
    center = c - cross(o - i, i - j).unit()*x_mag;
    return true;
  }

  //TODO pivot() function that takes two vertices, a mesh acceleration structure, and a radius rho
    //and computes the vertex which when touched will make ensure no other vertices are in the ball
    //Cannot fail on the inside half edge, as it is a well formed triangle via our seed or algorithm prior.
  bool pivot_from( HalfedgeIter inside_halfedge, double rho, VertexIter& populate, MeshResampler* meshR) {
    printf("pivotting from\n");
    Vector3D sigma_i = inside_halfedge->vertex()->position;
    Vector3D sigma_j = inside_halfedge->next()->vertex()->position;
    Vector3D m = 0.5*(sigma_i + sigma_j);
    Vector3D sigma_o = inside_halfedge->next()->next()->vertex()->position;
    Vector3D* c_ijo = new Vector3D();
    circumsphere_center(sigma_i, sigma_j, sigma_o, rho, *c_ijo);
    printf("Computed some basic info\n");

    /*First we must compute all the valid vertices for which the ball can touch
      v, sigma_i, and sigma_j without containing any other points. We must look in
      a 2*rho distance from m */

    printf("About to get neighbors\n");
    std::vector<VertexIter> rho_closest;
    rho_closest = (*meshR).get_neighbors(m);
    printf("Done\n");

    printf("Size of our acceleration Structure query is %d\n", rho_closest.size());

    std::vector<Vector3D*> candidate_centers;
    std::vector<VertexIter> candidate_vertices;
    for (VertexIter &v : rho_closest) {
      if ( (v->position.x != sigma_o.x || v->position.y != sigma_o.y || v->position.z != sigma_o.z) &&
       (v->position.x != sigma_i.x || v->position.y != sigma_i.y || v->position.z != sigma_i.z ) &&
        (v->position.x != sigma_j.x || v->position.y != sigma_j.y || v->position.z != sigma_j.z)) { //Cannot use the trivial third point in our triangle nor the existing i and j

        Vector3D *cx = new Vector3D();

        if (circumsphere_center(sigma_i, sigma_j, v->position, rho, *cx)) {
          // printf("\n");
          // printf("Sanity check that at least three points are touching here\n");
          // printf("rho: %4f\n", rho);
          // printf("center sphere: %4f %4f %4f\n", cx->x, cx->y, cx->z);
          // printf("sigma_i: %4f %4f %4f -> %4f\n", sigma_i.x, sigma_i.y, sigma_i.z, (sigma_i - *cx).norm() );
          // printf("sigma_j: %4f %4f %4f -> %4f\n", sigma_j.x, sigma_j.y, sigma_j.z, (sigma_j - *cx).norm());
          // printf("candidate vertex: %4f %4f %4f -> %4f\n", v->position.x, v->position.y, v->position.z, (v->position - *cx).norm());
          // printf("\n");
          bool valid_flag  = true;
          double dist_from_center;
          for (VertexIter other : rho_closest) {
            if (other->position.x == sigma_i.x && other->position.y == sigma_i.y && other->position.z == sigma_i.z ) {
              continue;
            } else if (other->position.x == sigma_j.x && other->position.y == sigma_j.y && other->position.z == sigma_j.z ) {
              continue;
            } else if (other->position.x == v->position.x && other->position.y == v->position.y && other->position.z == v->position.z ) {
               continue;
            }
            dist_from_center = (other->position - *cx).norm();
            if (dist_from_center < rho - 0.00001) {
              valid_flag = false; //flag that too we cannot use this vertex, as another vertex is inside the ball;
              break;
            }
          }

          if (valid_flag) { //Ball is touching three points exactly, add v to candidate centers
            // printf("We made a candidate vertex\n");
            candidate_centers.push_back(cx);
            // printf("Candidate center placed at %4f %4f %4f \n", cx->x, cx->y, cx->z);

            // circumsphere_center(sigma_i, sigma_j, v->position, rho, *cx);

            // printf("sigma_i at: %4f %4f %4f\n", sigma_i.x, sigma_i.y, sigma_i.z);
            // printf("sigma_j at: %4f %4f %4f\n", sigma_j.x, sigma_j.y, sigma_j.z);
            // printf("candidate vertex at: %4f %4f %4f\n", v->position.x, v->position.y, v->position.z);
            // printf("Candidate center sanity check at %4f %4f %4f \n", cx->x, cx->y, cx->z);
            candidate_vertices.push_back(v);
          }
        }
      }
    }

    /*At this point we have accumulated our candidate vertices to add and
      the center of the ball when it rolls on them in candidate_centers
      and candidate_vertices Now we compute the closest center along the
      3D circle gamma with radius rho centered at m*/

    if (candidate_centers.size() == 0) {
      return false;
    }

    printf("Starting here to find segfault\n");
    double best_t_sofar = 3*PI;
    VertexIter best_vertex_sofar;
    int i = 0;
    Vector3D m_cijo = *c_ijo - m;
    for (Vector3D* candidate_center : candidate_centers) {
      printf("Candidate center reached at %4f %4f %4f \n", candidate_center->x, candidate_center->y, candidate_center->z);
      Vector3D m_candidate = *candidate_center - m;
      double norm = (m_cijo.norm()*m_candidate.norm());
      double cos_theta = dot(m_cijo, m_candidate)/norm;
      double sin_theta = cross(m_cijo, m_candidate).norm()/norm;

      double candidate_t;

      printf("Computing distances...\n");
      if (sin_theta > 0.0) { /*top half*/
        candidate_t = acos(cos_theta);
      } else if (cos_theta > 0.0) { /*fourth quadrant*/
        candidate_t = 2*PI + asin(sin_theta);
      } else { /*Third quadrant*/
        candidate_t = PI - asin(sin_theta);
      }
      printf("Check\n");


      printf("Trying to match best_t with best_vertex_sofar\n");
      printf("3*PI is %4f and the candidate_t is %4f\n", 3*PI, candidate_t);
      if (candidate_t < best_t_sofar) {

        best_t_sofar = candidate_t;
        best_vertex_sofar = candidate_vertices[i];
        printf("We did it, populating with the vertex at %4f %4f %4f\n", best_vertex_sofar->position.x, best_vertex_sofar->position.y, best_vertex_sofar->position.z);
      }
      printf("Check\n");
      i++;
    }

    printf("Return okay..\n");
    populate = best_vertex_sofar;
    printf("check\n");
    return true;

  }

  bool MeshResampler::calculateBallPointDemo( Halfedge h, HalfedgeMesh& mesh, VertexIter& populate) {
    HalfedgeIter hIter = h.twin()->twin();
    double rho = 0.4;


    rho = set_rho(mesh, rho);

    return pivot_from(hIter, rho, populate, this);
  }


  // Constructs a plane from a collection of points
// so that the summed squared distance to all points is minimzized
bool normal_at_point(Vector3D point, std::vector<VertexIter> points, Vector3D& populate) {
    int n = points.size();
    if (n < 3) {
      printf("normal_at_point: Not enough points to compute normal\n");
      return false;
    }

    Vector3D sum = Vector3D();

    for (VertexIter v : points) {
        sum += v->position;
    }

    Vector3D centroid = sum * (1.0 / ((double) n));

    // Calc full 3x3 covariance matrix, excluding symmetries:
    double xx = 0.0;
    double xy = 0.0;
    double xz = 0.0;
    double yy = 0.0;
    double yz = 0.0;
    double zz = 0.0;

    for (VertexIter v : points) {
        Vector3D r = v->position - centroid;
        xx += r.x * r.x;
        xy += r.x * r.y;
        xz += r.x * r.z;
        yy += r.y * r.y;
        yz += r.y * r.z;
        zz += r.z * r.z;
    }

    double det_x = yy*zz - yz*yz;
    double det_y = xx*zz - xz*xz;
    double det_z = xx*yy - xy*xy;

    double det_max = max(det_x, max(det_y, det_z));
    if (det_max <= 0.0) {
      printf("%s\n", "The points don't span a plane");
      return false;
    }

    if (det_max == det_x) {
        double a = (xz*yz - xy*zz) / det_x;
        double b = (xy*yz - xz*yy) / det_x;
        populate = Vector3D(1.0, a, b).norm();
    } else if (det_max == det_y) {
        double a = (yz*xz - xy*zz) / det_y;
        double b = (xy*xz - yz*xx) / det_y;
        populate = Vector3D(a, 1.0,  b).norm();
    } else {
        double a = (yz*xy - xz*yy) / det_z;
        double b = (xz*xy - yz*xx) / det_z;
        populate = Vector3D(a, b, 1.0).norm();
    }
    return true;
}

  /*Assumes that each point has its normal populated already!*/
  bool make_normals_consistent(std::vector<VertexIter> points) {
    /**Make a decision to which direction it is facing, by computing the dot product of the plane normal
      *and each point to the point. We would like the normal to be facing the direction that minimizes points
      *outside of the surface, and thus in front of the plane
      */

    int n = points.size();
     if (n <= 1) {
      printf("make_normals_consistent: Not enough points to compute centroid\n");
      return false;
    }

    Vector3D sum = Vector3D();
    for (VertexIter v : points) {
        sum += v->position;
    }
    Vector3D centroid = sum * (1.0 / ((double) n));

    for (VertexIter v : points) {
      if (dot(v->norm, (v->position - centroid)) < 0.0) {
        v->norm = -1.0*(v->norm);
      }
    }
  }

  HalfedgeIter HalfedgeMesh::createSeedTriangle(VertexIter sigma, VertexIter  alpha, VertexIter beta) { /*This only gets called on three isolated vertices if the algorithm is run properly*/
    Vector3D avg_vertex_norm = sigma->norm + alpha->norm + beta->norm;

    HalfedgeIter o1 = this->newHalfedge();
    HalfedgeIter o2 = this->newHalfedge();
    HalfedgeIter o3 = this->newHalfedge();
    HalfedgeIter i1 = this->newHalfedge();
    HalfedgeIter i2 = this->newHalfedge();
    HalfedgeIter i3 = this->newHalfedge();

    EdgeIter e1 = this->newEdge();
    EdgeIter e2 = this->newEdge();
    EdgeIter e3 = this->newEdge();

    FaceIter f = this->newFace();

    if (dot(avg_vertex_norm, cross( (beta->position - sigma->position), (sigma->position - alpha->position))) >= 0.0) {/*counterclockwise is alpha->sigma->beta, return the halfedge beta->alpha*/
      o1->setNeighbors( o3,
                        i1,
                        beta,
                        e1,
                        f);

      o2->setNeighbors( o1,
                        i2,
                        alpha,
                        e2,
                        f);

      o3->setNeighbors( o2,
                        i3,
                        sigma,
                        e3,
                        f);

      i1->setNeighbors( i2,
                        o1,
                        sigma,
                        e1,
                        f);

      i2->setNeighbors( i3,
                        o2,
                        beta,
                        e2,
                        f);

      i3->setNeighbors( i1,
                        o3,
                        alpha,
                        e3,
                        f);
      e1->halfedge() = i1;
      e2->halfedge() = i2;
      e3->halfedge() = i3;
      f->halfedge() = i1;
      sigma->halfedge() = i1;
      beta->halfedge() = i2;
      alpha->halfedge() = i3;
      return i2;

    } else { /*counterclockwise is beta->sigma->alpha, return the halfedge alpha->beta*/
      o1->setNeighbors( o3,
                        i1,
                        alpha,
                        e1,
                        f);

      o2->setNeighbors( o1,
                        i2,
                        beta,
                        e2,
                        f);

      o3->setNeighbors( o2,
                        i3,
                        sigma,
                        e3,
                        f);

      i1->setNeighbors( i2,
                        o1,
                        sigma,
                        e1,
                        f);

      i2->setNeighbors( i3,
                        o2,
                        alpha,
                        e2,
                        f);

      i3->setNeighbors( i1,
                        o3,
                        beta,
                        e3,
                        f);
      e1->halfedge() = i1;
      e2->halfedge() = i2;
      e3->halfedge() = i3;
      f->halfedge() = i1;
      sigma->halfedge() = i1;
      beta->halfedge() = i3;
      alpha->halfedge() = i2;
      return i2;
    }
  }

  bool computeSeedTriangleGivenPoint( VertexIter sigma, std::vector<VertexIter> accel_struct, double rho, VertexIter& populate_alpha, VertexIter& populate_beta) {
    if (accel_struct.size() < 2) {
      printf("computeSeedTriangleGivenPoint: Cannot compute seed triangle with less than 2 free vertices\n");
      return false;
    }
    for (VertexIter sigma_alpha : accel_struct) {

      if (sigma_alpha->position.x != sigma->position.x ||
          sigma_alpha->position.y != sigma->position.y ||
          sigma_alpha->position.z != sigma->position.z) { /*We cannot use sigma as two points on the triangle*/

        for (VertexIter sigma_beta : accel_struct) {

          if ((sigma_beta->position.x != sigma->position.x || sigma_beta->position.y != sigma->position.y || sigma_beta->position.z != sigma->position.z)
            && (sigma_beta->position.x != sigma_alpha->position.x || sigma_beta->position.y != sigma_alpha->position.y || sigma_beta->position.z != sigma_alpha->position.z)) { /*We cannot use sigma or sigma_alpha as two points on the triangle*/

            Vector3D ball_center;
            Vector3D normal_orientation;
            bool orientable = false;
            if (circumsphere_center(sigma->position , sigma_alpha->position, sigma_beta->position, rho, ball_center)) { /*A sphere can be computed from these three*/
              normal_orientation = sigma->norm + sigma_alpha->norm + sigma_beta->norm;
              if (dot(normal_orientation, ball_center - ((sigma->position + sigma_alpha->position + sigma_beta->position) * 1.0/3.0)) > 0.0)  {
                orientable = true;
              } else if (circumsphere_center(sigma->position , sigma_beta->position, sigma_alpha->position, rho, ball_center)) {
              /*The normal of the computed ball and triangle did not orient with the vertex normals, compute the counterpart ball isntead*/
                orientable = true;
              } else {
                orientable = false;
              }
            }

            if (orientable) {/*We proceed with the ball center populated, and compute if this is a valid configuration*/
              bool valid_flag  = true;
              double dist_from_center;
              for (VertexIter other : accel_struct) {
                /*Ignore the trivial three points intersection*/
                if (other->position.x == sigma->position.x && other->position.y == sigma->position.y && other->position.z == sigma->position.z ) {
                  continue;
                } else if (other->position.x == sigma_alpha->position.x && other->position.y == sigma_alpha->position.y && other->position.z == sigma_alpha->position.z ) {
                  continue;
                } else if (other->position.x == sigma_beta->position.x && other->position.y == sigma_beta->position.y && other->position.z == sigma_beta->position.z ) {
                  continue;
                }
                dist_from_center = (other->position - ball_center).norm();
                if (dist_from_center < rho) {
                  valid_flag = false; //flag that too we cannot use this vertex, as another vertex is inside the ball;
                  break;
                }
              }
              if (valid_flag) {/*Success! we found a well formed seed triangle with good orientation*/
                /*Now we must return the two vertices to form a triangle*/
                populate_alpha = sigma_alpha;
                populate_beta = sigma_beta;
                return true;
              }
            }
          }
        }
      }
    }
    return false;
  }

  HalfedgeIter MeshResampler::ball_pivot( HalfedgeMesh& mesh) {
    double rho = 0.25;
    //TODO clear everything!
    // for( EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++ ) {
    //   mesh.deleteEdge(e);
    // }

    // for( HalfedgeIter e = mesh.halfedgesBegin(); e != mesh.halfedgesBegin(); e++ ) {
    //   mesh.deleteHalfedge(e);
    // }

    // for( FaceIter e = mesh.facesBegin(); e != mesh.facesEnd(); e++ ) {
    //   mesh.deleteFace(e);
    // }


    //TODO compute the radius list we want by doing some statistics on the points
    //TODO build a mesh acceleration structure (voxel grid), so that we can look at a vertices nearest neighbors
    //TODO Iterating from smallest rho to largest
      std::vector<VertexIter> dummy_accel_struct;
      std::deque<VertexIter> unused_vertices;
      std::deque<EdgeIter> active_edges;
      std::deque<EdgeIter> front_edges;

      //TODO create a queue of free vertices
      for( VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++ ) {
        v->BPisUsed = false;
        unused_vertices.push_back(v);
        dummy_accel_struct.push_back(v);
      }

      //TODO Loop on compute a seed triangle
      VertexIter candidate_sigma;
      bool seedFound = false;
      VertexIter sigma_alpha;
      VertexIter sigma_beta;
      while (!(seedFound) && unused_vertices.size() > 0) {
        bool candidate_found = false;
        /*Use voxel accel struct to get the closest points here*/
        while (unused_vertices.size() > 0) {
          candidate_sigma = unused_vertices.front();
          printf("Candidate suggested\n");
          unused_vertices.pop_front();
          if ((candidate_sigma->BPisUsed)) {
            printf("But continued\n");
            continue;
          } else {
            candidate_found = true;
            break;
          }
        }

        if (candidate_found) {
          printf("Candidate_found\n");
          seedFound = computeSeedTriangleGivenPoint(candidate_sigma, dummy_accel_struct, rho, sigma_alpha, sigma_beta);
        }
      }

      if (seedFound) {
        mesh.createSeedTriangle(candidate_sigma, sigma_alpha, sigma_beta);
        EdgeIter e1 = candidate_sigma->halfedge()->edge();
        EdgeIter e2 = sigma_alpha->halfedge()->edge();
        EdgeIter e3 = sigma_beta->halfedge()->edge();
        active_edges.push_back(e1);
        active_edges.push_back(e2);
        active_edges.push_back(e3);
        front_edges.push_back(e1);
        front_edges.push_back(e2);
        front_edges.push_back(e3);
        e1->BPisActive = true;
        e1->BPisBoundary = false;
        e2->BPisActive = true;
        e2->BPisBoundary = false;
        e3->BPisActive = true;
        e3->BPisBoundary = false;

        printf("Candidate Sigma is here: %4f %4f %4f \n", candidate_sigma->position.x, candidate_sigma->position.y, candidate_sigma->position.z);
        printf("returned correctly\n");
      } else {
        /*We must try another rho or kill the floating vertices and call it done*/
      }

      //TODO compute the "active" edge e, or edge on the fringe that we must pivot over
      bool active_edge_found = false;
      EdgeIter candidate_active_edge;

      /*Use voxel accel struct to get the closest points here*/
      while (active_edges.size() > 0) {
        candidate_active_edge = active_edges.front();
        printf("Active Edge Candidate suggested\n");
        active_edges.pop_front();
        if ((candidate_active_edge->BPisActive)) {
          candidate_found = true;
          break;
        } else {
          printf("But continued\n");
          continue;
        }
      }

      if (active_edge_found) {
          printf("Active Edge Candidate_found\n");
      

        //TODO not_used(), not_internal
        // 3. if (Vertex k = pivot(e) && ( not_used(k) || not_internal(k) ) )
          VertexIter k;
          if (pivot_from( candidate_active_edge, rho, k, this)) {
            if (!(k.BPisUsed) || (k->halfedge()->edge()->BPisActive || k->halfedge()->edge()->BPisBoundary)) {

            }

          }


          //TODO, function that reassigns half edge pointers, edge pointers, face pointers, to make a triangle
          // 4. output triangle(i,  k , j )


          //TODO join(e, ek1, ek2), which takes in an old edge, and two new edges, marks the old as internal and the new as FRONT, luckily for us, we do not need to worry about glueing 
            //here because a vertex will just overwrite its edge pointers, and half edges are expected to be opposite facing
          // 5. join(e(i,j) , k , F)


        // 8 . else

          // TODO function mark an edge as a fixed boundary 
          // 9 . mark as boundary(e(i;j))
      }

    Vector3D i = Vector3D(123, 456 , 789);
    Vector3D j = Vector3D(666,66,6);
    Vector3D o = Vector3D(34, 43.24, -3422.0321);
    Vector3D *c = new Vector3D();

    circumsphere_center( i, j, o, 234556, *c);

    printf("Circumsphere center is : %4f %4f %4f\n", c->x, c->y, c->z);
    printf("Distance from i is : %4f\n", (i-*c).norm());
    printf("Distance from j is : %4f\n", (j-*c).norm());
    printf("Distance from o is : %4f\n", (o-*c).norm());


    circumsphere_center( i, o, j, 234556, *c);

    printf("Circumsphere center is : %4f %4f %4f\n", c->x, c->y, c->z);
    printf("Distance from i is : %4f\n", (i-*c).norm());
    printf("Distance from j is : %4f\n", (j-*c).norm());
    printf("Distance from o is : %4f\n", (o-*c).norm());
    return candidate_sigma->halfedge();

  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // Each vertex and edge of the original surface can be associated with a vertex in the new (subdivided) surface.
    // Therefore, our strategy for computing the subdivided vertex locations is to *first* compute the new positions
    // using the connectity of the original (coarse) mesh; navigating this mesh will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse. We will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.


    // TODO Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // TODO and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // TODO a vertex of the original mesh.
    for( VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++ ) {
      v->isNew = false;

      int n = 0; // degree
      Vector3D pSum = Vector3D(0., 0., 0.);

      // iterate over halfedges incident on this vertex
      HalfedgeIter h = v->halfedge();
      VertexIter neighbor;
      do
      {
        n++; // increment degree
        neighbor = h->vertex();
        if (neighbor == v) {
          neighbor = h->twin()->vertex();
        }
        pSum += neighbor->position;
        // move to the next halfedge around the vertex
        h = h->twin()->next();

      } while( h != v->halfedge() ); // done iterating over halfedges
      if (v->isBoundary()) {
        n++; //increment degree by one more
        pSum += v->position;
      }

      float u = 3.0/16.0;
      if (n != 3) {
       u = 3.0 / (8.0*(float)n);
      }
      v->newPosition = (1.0 - (float)n*u) * v->position + u * pSum;

    }

    // TODO Next, compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
    for( EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++ ) {
      e->isNew = false;


      if (e->halfedge()->face()->isBoundary()) {
        /*Edge is pointing to the boundary halfedge*/
        e->newPosition = (3.5/8.0)*(e->halfedge()->vertex()->position +
        e->halfedge()->twin()->vertex()->position)   +
        (e->halfedge()->twin()->next()->next()->vertex()->position)/8.0;

      } else if (e->halfedge()->twin()->face()->isBoundary()) {
        /*Edge is pointing to the non boundary halfedge*/
        e->newPosition = (3.5/8.0)*(e->halfedge()->vertex()->position +
        e->halfedge()->twin()->vertex()->position)   +
        (e->halfedge()->next()->next()->vertex()->position)/8.0;

      } else {
        e->newPosition = (3.0/8.0)*(e->halfedge()->vertex()->position +
        e->halfedge()->twin()->vertex()->position)   +

        (e->halfedge()->next()->next()->vertex()->position +
        e->halfedge()->twin()->next()->next()->vertex()->position)/8.0;
      }


    }

    // TODO Next, we're going to split every edge in the mesh, in any order.  For future
    // TODO reference, we're also going to store some information about which subdivided
    // TODO edges come from splitting an edge in the original mesh, and which edges are new,
    // TODO by setting the flat Edge::isNew.  Note that in this loop, we only want to iterate
    // TODO over edges of the original mesh---otherwise, we'll end up splitting edges that we
    // TODO just split (and the loop will never end!)

    for( EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++ ) {

        if (!e->halfedge()->vertex()->isNew && !e->halfedge()->twin()->vertex()->isNew) {
          VertexIter newV = mesh.splitEdge( e );
          newV->newPosition = e->newPosition;
          //newV->newPosition = newV->position;
        }
    }

    // TODO Now flip any new edge that connects an old and new vertex.
    for( EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++ ) {
      if (e->isNew) {
        VertexIter a = e->halfedge()->vertex();
        VertexIter b = e->halfedge()->twin()->vertex();
        if (a->isNew != b->isNew) {
          mesh.flipEdge( e );
        }
      }
    }

    // TODO Finally, copy the new vertex positions into final Vertex::position.
    for( VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++ ) {
      v->position = v->newPosition;
    }
  }
}
