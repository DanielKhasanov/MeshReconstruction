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
      // printf("Cannot flip a boundary edge.\n" );
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


      // printf("%f, %f, %f: \n" , middleV->position[0], middleV->position[1], middleV->position[2]);
      return middleV;
    }
  }


  double MeshResampler::set_rho(HalfedgeMesh& mesh, double newRho, bool set) {
    printf("Setting rho\n");
    bool flag = true;
    bool freePass = false;
    bool flag2 = true;
    double limit = 40;
    rho = newRho;
    // printf("Rho is now %4f\n", rho);
    double count;
    double n;
    if (set) {
      for (const auto &entry : map) {
        delete(entry.second);
      }
      // printf("Deleted entries\n");
      map.clear();


      // printf("Iterating...\n");
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
          if (pos.y > y_max) {y_max = pos.y;}
          if (pos.z > z_max) {z_max = pos.z;}
        }
      }
      // printf("Done iterating\n");
      double x_diff = x_max - x_min;
      double y_diff =  y_max - y_min;
      double z_diff =  z_max - z_min;

      // printf("Doing some if cases\n");
      if (x_diff > y_diff) {
        mod = x_diff;
      } else {
        mod = y_diff;
      }
      if (z_diff > mod) {
        mod = z_diff;
      }
      // printf("Those are done\n");



      int num_bins = max((int) (mod/(2.0*rho)), 1) ;
      // printf("The number of bins is %d\n", num_bins);

      for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
        Vector3D p = v->position;
        int w_index = (int) ((p.x - x_min)/ (2*rho));
        int h_index = (int) ((p.y - y_min)/ (2*rho));
        int t_index = (int) ((p.z - z_min)/ (2*rho));

        // printf("wi, hi, ti is %d %d %d\n",w_index, h_index, t_index );
        int hash = num_bins * num_bins * w_index + num_bins * h_index + t_index;
        // printf("Computed hash is %d\n", hash);
        if (map.find(hash) == map.end()) {
          std::vector<VertexIter > * vec = new std::vector<VertexIter >();
          map[hash] = vec;
        }
        map[hash]->push_back(v);
      }
      return rho;
    }
    do {
      flag2 = true;
      for (const auto &entry : map) {
        delete(entry.second);
      }
      // printf("Deleted entries\n");
      map.clear();


      // printf("Iterating...\n");
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
          if (pos.y > y_max) {y_max = pos.y;}
          if (pos.z > z_max) {z_max = pos.z;}
        }
      }
      // printf("Done iterating\n");
      double x_diff = x_max - x_min;
      double y_diff =  y_max - y_min;
      double z_diff =  z_max - z_min;

      // printf("Doing some if cases\n");
      if (x_diff > y_diff) {
        mod = x_diff;
      } else {
        mod = y_diff;
      }
      if (z_diff > mod) {
        mod = z_diff;
      }
      // printf("Those are done\n");



      int num_bins = max((int) (mod/(2.0*rho)), 1) ;
      // printf("The number of bins is %d\n", num_bins);

      for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
        Vector3D p = v->position;
        int w_index = (int) ((p.x - x_min)/ (2*rho));
        int h_index = (int) ((p.y - y_min)/ (2*rho));
        int t_index = (int) ((p.z - z_min)/ (2*rho));

        // printf("wi, hi, ti is %d %d %d\n",w_index, h_index, t_index );
        int hash = num_bins * num_bins * w_index + num_bins * h_index + t_index;
        // printf("Computed hash is %d\n", hash);
        if (map.find(hash) == map.end()) {
          std::vector<VertexIter > * vec = new std::vector<VertexIter >();
          map[hash] = vec;
        }
        map[hash]->push_back(v);
      }
      count = 0.0;
      n = 0;
      int min = 0;
      for (const auto &entry : map) {
        // printf("voxel {%d} has %zu entries\n", entry.first, entry.second->size());
        count += entry.second->size();
        n++;
        if (n == 1 || entry.second->size() < min) {
          min = entry.second->size();
        }
        if (entry.second->size() < 3) {
          // printf("Voxel %d has less than 3 vertices\n", entry.first);
          // for (VertexIter v : *entry.second) {
          //   Vector3D vec = v->position;
          //   printf("Locations at %4f %4f %4f\n", vec.x, vec.y, vec.z);
          // }
          int sum = 0;
          for (int i = -1; i <=  1; i +=  1) {
            for (int j = -1; j <=  1; j += 1) {
              for (int k = -1; k <=  1; k += 1) {

                int next_hash = entry.first + num_bins * num_bins * i + num_bins * j + k;
                // printf("next hash is {%d %d %d} %d\n", i,j,k, next_hash);
                if (map.find(next_hash) != map.end()) {
                  sum += map[next_hash]->size();
                }
              }
            }
          }
          if (sum < 3) {
            freePass = true;
            flag2 = false;
          }
        }
      }
      // printf("Rho is currently %4f\n", rho);
      if ((freePass && flag2) || (double) count/(double) n < limit) {
        flag = false;
      } else if (freePass) {
        rho = rho * 1.2;
      } else {
        rho = rho * 0.7;
      }
      // printf("Average number of vertices per voxel: %4f\n",(double) count/(double) n );

      // printf("Smallest voxel had %d vertices\n", min);
      // printf("number of voxels is %f\n", n);
    } while (flag);


    // int count = 0;
    // for (const auto &entry : map) {
    //   // printf("voxel {%d} has %zu entries\n", entry.first, entry.second->size());
    //   count++;
    // }
    printf("Rho is set to %4f\n", rho);
    printf("Average number of vertices per voxel: %4f\n",(double) count/(double) n );

    return rho;
  }

  std::vector<VertexIter> MeshResampler::get_neighbors(Vector3D p) {
    // printf("getting neighbors\n");
    int num_bins = max((int) (mod/(2.0*rho)), 1);
    // printf("There are %d bins given mod %4f and 2r %4f\n",num_bins, mod, 2*rho );

    int w_index = (int) ((p.x - x_min)/ (2*rho));
    int h_index = (int) ((p.y - y_min)/ (2*rho));
    int t_index = (int) ((p.z - z_min)/ (2*rho));


    // printf("QUERY: wi, hi, ti is %d %d %d\n",w_index, h_index, t_index );
    int hash = num_bins * num_bins * w_index + num_bins * h_index + t_index;
    if (map.find(hash) != map.end()) {
     // printf("QUERY: Computed hash is %d with size %zu\n",  hash , (*map[hash]).size());
    }

    std::vector<VertexIter>  * vec = new std::vector<VertexIter >();
    for (int i = max(0, w_index - 1); i <=  min(num_bins, w_index + 1); i +=  1) {
      for (int j = max(0, h_index - 1); j <=  min(num_bins, h_index + 1); j += 1) {
        for (int k = max(0, t_index - 1); k <=  min(num_bins , t_index + 1); k++) {

          int next_hash = num_bins * num_bins * i + num_bins * j + k;
          // printf("next hash is {%d %d %d} %d\n", i,j,k, next_hash);
          if (map.find(next_hash) != map.end()) {
            // printf("Pushing back voxel at %d %d %d\n",i,j,k );
            // printf("To get a hash value %d with voxel size %zu \n",next_hash, (*map[next_hash]).size() );
            for (VertexIter ver : *map[next_hash]) {
              vec->push_back(ver);
            }
          }
        }
      }
    }
    // printf("done\n");
    // printf("Sorting neighbors\n");
    std::sort(vec->begin(), vec->end(), [&p] (const VertexIter lhs, const VertexIter rhs ){  return (lhs->position - p).norm() < (rhs->position - p).norm();});

    // printf("Done\n");
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

    if (i.x == j.x && i.y == j.y && i.z == j.z) {
      printf("Circumsphere center: failed because points nonunique\n");
      return false;
    } else if (i.x == o.x && i.y == o.y && i.z == o.z) {
      printf("Circumsphere center: failed because points nonunique\n");
      return false;
    } else if (j.x == o.x && j.y == o.y && j.z == o.z) {
      printf("Circumsphere center: failed because points nonunique\n");
      return false;
    }
    Vector3D c = circumcenter(i, j, o);
    double alpha_sqr = (c - i).norm2();
    double discriminant = (rho*rho) - alpha_sqr;
    if (discriminant < 0.0) {
      return false;
    }
    double x_mag = sqrt(discriminant);

    center = c - cross(o - i, i - j).unit()*x_mag;
    // printf("c is the vector %4f %4f %4f\n", c.x,c.y,c.z);
    // printf("cross is the vector %4f %4f %4f\n", cross(o - i, i - j).x, cross(o - i, i - j).y,cross(o - i, i - j).z);
    // printf("center is the vector %4f %4f %4f\n", center.x,center.y,center.z);
    return true;
  }

  //TODO pivot() function that takes two vertices, a mesh acceleration structure, and a radius rho
    //and computes the vertex which when touched will make ensure no other vertices are in the ball
    //Cannot fail on the inside half edge, as it is a well formed triangle via our seed or algorithm prior.
  bool pivot_from( HalfedgeIter inside_halfedge, double rho, VertexIter& populate, MeshResampler* meshR) {
    // printf("pivoting from an inside halfedge\n");
    Vector3D sigma_i = inside_halfedge->vertex()->position;
    Vector3D sigma_j = inside_halfedge->next()->vertex()->position;
    Vector3D m = 0.5*(sigma_i + sigma_j);
    Vector3D sigma_o = inside_halfedge->next()->next()->vertex()->position;
    Vector3D c_ijo;

    // printf("SURELY THIS WONT FAIL\n");
    if (!circumsphere_center(sigma_i, sigma_j, sigma_o, rho, c_ijo)) {
      printf("BUT LO AND BEHOLD\n");
    }

    if (dot(inside_halfedge->vertex()->norm + inside_halfedge->next()->vertex()->norm + inside_halfedge->next()->next()->vertex()->norm, c_ijo - ((sigma_i + sigma_j + sigma_o) * 1.0/3.0)) < 0.0)  {
            // printf("Orientable originally\n");
      circumsphere_center(sigma_i, sigma_o, sigma_j, rho, c_ijo);
    }
          

    // printf("cijo is the vector %4f %4f %4f\n", (c_ijo).x, (c_ijo).y, (c_ijo).z);
    // printf("m is the vector %4f %4f %4f\n", m.x, m.y, m.z);
    Vector3D m_cijo = c_ijo - m;
    // printf("m_cijo is the vector %4f %4f %4f\n", m_cijo.x, m_cijo.y,m_cijo.z);

    // printf("Computed some basic info\n");

    /*First we must compute all the valid vertices for which the ball can touch
      v, sigma_i, and sigma_j without containing any other points. We must look in
      a 2*rho distance from m */

    // printf("About to get neighbors\n");
    std::vector<VertexIter> rho_closest;
    rho_closest = (*meshR).get_neighbors(m);
    // printf("Done\n");

    printf("Size of our acceleration Structure query is %d\n", rho_closest.size());

    std::vector<Vector3D> candidate_centers;
    std::vector<VertexIter> candidate_vertices;
    for (VertexIter &v : rho_closest) {
      // printf("roll call: %4f %4f %4f\n", v->position.x, v->position.y, v->position.z);
      if ( (v->position.x != sigma_o.x || v->position.y != sigma_o.y || v->position.z != sigma_o.z) &&
       (v->position.x != sigma_i.x || v->position.y != sigma_i.y || v->position.z != sigma_i.z ) &&
        (v->position.x != sigma_j.x || v->position.y != sigma_j.y || v->position.z != sigma_j.z)) { //Cannot use the trivial third point in our triangle nor the existing i and j

        Vector3D cx;

        if (circumsphere_center(sigma_i, sigma_j, v->position, rho, cx)) {
          // printf("\n");
          // printf("Sanity check that at least three points are touching here\n");
          // printf("rho: %4f\n", rho);
          // printf("center sphere: %4f %4f %4f\n", cx->x, cx->y, cx->z);
          // printf("sigma_i: %4f %4f %4f -> %4f\n", sigma_i.x, sigma_i.y, sigma_i.z, (sigma_i - *cx).norm() );
          // printf("sigma_j: %4f %4f %4f -> %4f\n", sigma_j.x, sigma_j.y, sigma_j.z, (sigma_j - *cx).norm());
          // printf("candidate vertex: %4f %4f %4f -> %4f\n", v->position.x, v->position.y, v->position.z, (v->position - *cx).norm());
          // printf("\n");

          bool valid_flag = true;
          if (!(dot(v->norm, cx - ((sigma_i + sigma_j + v->position) * 1.0/3.0)) > 0.0))  {
            // printf("Orientable originally\n");
              if (!(circumsphere_center(sigma_i , v->position, sigma_j, rho, cx))) {
              // printf("Non Orientable when flipped\n");
              /*The normal of the computed ball and triangle did not orient with the vertex normals, compute the counterpart ball isntead*/
                valid_flag = false;
                break;
              }
          }

          // printf("Orientation cehck: %4f\n", dot(v->norm, cx - ((sigma_i + sigma_j + v->position) * 1.0/3.0)) );

          double dist_from_center;
          for (VertexIter other : rho_closest) {
            if (other->position.x == sigma_i.x && other->position.y == sigma_i.y && other->position.z == sigma_i.z ) {
              continue;
            } else if (other->position.x == sigma_j.x && other->position.y == sigma_j.y && other->position.z == sigma_j.z ) {
              continue;
            } else if (other->position.x == v->position.x && other->position.y == v->position.y && other->position.z == v->position.z ) {
               continue;
            }
            dist_from_center = (other->position - cx).norm();
            if (dist_from_center < rho - 0.00001) {
              // printf("A vertex is inside the containing sphere\n");
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
      printf("No candidate centers found.\n");
      return false;
    }

    // printf("Starting here to find segfault\n");
    double best_t_sofar = 3*PI;
    double cos_theta_cache = -1;
    double sin_theta_cache = -1;
    VertexIter best_vertex_sofar;
    int i = 0;
    // printf("cijo is the vector %4f %4f %4f\n", (c_ijo).x, (c_ijo).y, (c_ijo).z);

    // printf("m is the vector %4f %4f %4f\n", m.x, m.y, m.z);
    // Vector3D m_cijo = c_ijo - m;
    // printf("m_cijo is the vector %4f %4f %4f\n", m_cijo.x, m_cijo.y,m_cijo.z);
    for (Vector3D candidate_center : candidate_centers) {
      // printf("Candidate center reached at %4f %4f %4f \n", candidate_center->x, candidate_center->y, candidate_center->z);

      Vector3D m_candidate = candidate_center - m;
      double norm = (m_cijo.norm()*m_candidate.norm());
      double cos_theta = max(-0.9999, min(0.9999, dot(m_cijo, m_candidate)/norm )) ;
      double sin_theta = max(-0.9999, min(0.9999, cross(m_cijo, m_candidate).norm()/norm));

      double candidate_t;

      // printf("Computing distances...\n");
      if (sin_theta > 0.0 + 0.000001) { /*top half*/

        // printf("top half\n");
        candidate_t = acos( cos_theta);
        // printf("Trying to match best_t with best_vertex_sofar\n");
        // printf("3*PI is %4f and the candidate_t is %4f, with sin_theta:%4f cos: %4f\n", 3*PI, candidate_t, sin_theta, cos_theta);
      } else if (cos_theta > 0.0 + 0.000001) { /*fourth quadrant*/
        
        candidate_t = 2*PI + asin(sin_theta);
      } else { /*Third quadrant*/
        return false;
        candidate_t = PI - asin(sin_theta);
      }
      // printf("Check\n");


      
      if (candidate_t < best_t_sofar) {

        best_t_sofar = candidate_t;
        cos_theta_cache = cos_theta;
        sin_theta_cache = sin_theta;
        best_vertex_sofar = candidate_vertices[i];
    //     printf("this one is inside\n");
    //         printf("%p\n",populate);
    // printf("%p\n", populate->halfedge());
        // printf("We did it, populating with the vertex at %4f %4f %4f\n", best_vertex_sofar->position.x, best_vertex_sofar->position.y, best_vertex_sofar->position.z);
      }

      i++;
    }

    // printf("Pivot: return okay\n");
    // printf("3*PI is %4f and the candidate_t is %4f, with sin_theta:%4f cos: %4f\n", 3*PI, best_t_sofar, sin_theta_cache, cos_theta_cache);
    populate = best_vertex_sofar;
    // printf("check\n");
    return true;

  }

  bool MeshResampler::calculateBallPointDemo( Halfedge h, HalfedgeMesh& mesh, VertexIter& populate) {
    HalfedgeIter hIter = h.twin()->twin();

    // set_rho(mesh, 0.1);
    // cluster_vertices(mesh);
    bool rv; 
    rv = pivot_from(hIter, rho, populate, this);

    // printf("%p\n",populate);
    // printf("%p\n", populate->halfedge());
    return rv;
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

    if (det_max == det_x || true) {
        double a = (xz*yz - xy*zz) / det_x;
        double b = (xy*yz - xz*yy) / det_x;
        populate = Vector3D(1.0, a, b).unit();
        // populate = Vector3D(1,0,0);
        // printf("detxyz: %4f %4f %4f \n", det_x, det_y, det_z);
        // printf("returning the plane %4f %4f %4f\n", populate.x, populate.y, populate.z);
    } else if (det_max == det_y && false) {
        double a = (yz*xz - xy*zz) / det_y;
        double b = (xy*xz - yz*xx) / det_y;

        populate = Vector3D(a, 1.0, b).unit();
                // populate = Vector3D(1,0,0);

        // printf("B\n");
        // printf("returning the plane %4f %4f %4f\n", populate.x, populate.y, populate.z);
    } else {
        double a = (yz*xy - xz*yy) / det_z;
        double b = (xz*xy - yz*xx) / det_z;


        // populate = Vector3D(1,0,0);
        populate = Vector3D(a, b, 1.0).unit();
        // printf("C\n");
        // printf("returning the plane %4f %4f %4f\n", populate.x, populate.y, populate.z);
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
      // printf("make_normals_consistent: Not enough points to compute centroid\n");
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
  unordered_map<int, vector<VertexIter > *> MeshResampler::cluster_vertices (HalfedgeMesh& mesh, double n) {
    double r = mod/n;
    int count = 0;
    unordered_map<int, vector<VertexIter > *> m;
    int c = 0;
    int num_bins = max((int) (mod/(2.0*r)), 1) ;
    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
      c++;
      if (c >= 20) {
        break;
      }
      Vector3D p = v->position;
      int w_index = (int) ((p.x - x_min)/ (r));
      int h_index = (int) ((p.y - y_min)/ (r));
      int t_index = (int) ((p.z - z_min)/ (r));

      // printf("wi, hi, ti is %d %d %d\n",w_index, h_index, t_index );
      int hash = num_bins * num_bins * w_index + num_bins * h_index + t_index;
      // printf("Computed hash is %d\n", hash);

      if (m.find(hash) == m.end()) {
        std::vector<VertexIter > * vec = new std::vector<VertexIter >();
        m[hash] = vec;
        count++;
      }

      m[hash]->push_back(v);
    }
    if (c < 20) {
      return m;
    }
    while (count < 20) {
      for (const auto &entry : m) {
        delete(entry.second);
      }
      // printf("Deleted entries\n");
      m.clear();
      count = 0;
      r = r/2.0;
      int num_bins = max((int) (mod/(2.0*r)), 1) ;
      // printf("The number of bins is %d\n", num_bins);

      for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
        Vector3D p = v->position;
        int w_index = (int) ((p.x - x_min)/ (r));
        int h_index = (int) ((p.y - y_min)/ (r));
        int t_index = (int) ((p.z - z_min)/ (r));

        // printf("wi, hi, ti is %d %d %d\n",w_index, h_index, t_index );
        int hash = num_bins * num_bins * w_index + num_bins * h_index + t_index;
        // printf("Computed hash is %d\n", hash);

        if (m.find(hash) == m.end()) {
          std::vector<VertexIter > * vec = new std::vector<VertexIter >();
          m[hash] = vec;
          count++;
        }

        m[hash]->push_back(v);
      }
      // printf("%d and %4f\n", count, r);
    }
    // printf("The number of large voxels is %d\n", count);
    // printf("The rho used is %4f\n", r);
    return m;
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

    FaceIter boundaryFace1 = this->newBoundary();
    FaceIter boundaryFace2 = this->newBoundary();
    FaceIter boundaryFace3 = this->newBoundary();

    boundaryFace1->halfedge() = o1;
    boundaryFace2->halfedge() = o2;
    boundaryFace3->halfedge() = o3;


    if (dot(avg_vertex_norm, cross( (beta->position - sigma->position), (sigma->position - alpha->position))) < 0.0) {/*counterclockwise is alpha->sigma->beta, return the halfedge beta->alpha*/
      o1->setNeighbors( o3,
                        i1,
                        beta,
                        e1,
                        boundaryFace1);

      o2->setNeighbors( o1,
                        i2,
                        alpha,
                        e2,
                        boundaryFace2);

      o3->setNeighbors( o2,
                        i3,
                        sigma,
                        e3,
                        boundaryFace3);

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
                        boundaryFace1);

      o2->setNeighbors( o1,
                        i2,
                        beta,
                        e2,
                        boundaryFace2);

      o3->setNeighbors( o2,
                        i3,
                        sigma,
                        e3,
                        boundaryFace3);

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

  bool HalfedgeMesh::createFrontTriangle(HalfedgeIter insideFront, VertexIter k, FaceIter& populate) {
    /*In the case that k is not_used, we extend the front to include it
      in the case that k is on the front, we must check if it is already somehow connected to a or b
      If it is connected to one, we choose the to be internal halfedge. the predecessor will join on
      the outside of the triangle. If both are connected, we simply assign a face*/

    printf("Creating a front triangle\n");
    VertexIter beta = insideFront->vertex();
    beta->halfedge() = insideFront;
    VertexIter alpha = insideFront->next()->vertex();
    alpha->halfedge() = insideFront->next();
    printf("gathered data\n");
    if (!(k->BPisUsed)) {/*Create four new halfedges*/
      printf("Adding isolated vertex\n");
      HalfedgeIter o1 = this->newHalfedge();
      HalfedgeIter o2 = this->newHalfedge();
      HalfedgeIter i1 = this->newHalfedge();
      HalfedgeIter i2 = this->newHalfedge();
      EdgeIter e1 = this->newEdge();
      e1->BPisActive = true;
      EdgeIter e2 = this->newEdge();
      e2->BPisActive = true;
      e1->halfedge() = i1;
      e2->halfedge() = i2;
      FaceIter f = this->newFace();
      f->halfedge() = i1;
      k->halfedge() = i1;

      FaceIter boundaryFace1 = insideFront->twin()->face();
      FaceIter boundaryFace2 = this->newBoundary();
      boundaryFace1->halfedge() = o1;
      boundaryFace2->halfedge() = o2;

      /*Compute a's predecessor*/
      HalfedgeIter a_pred;
      bool found_apred = false;
      HalfedgeIter h = alpha->halfedge();
      int maxIter = 10;
      do {
        if (h->twin()->next() == insideFront->twin()) {
          a_pred = h->twin();
          found_apred = true;
          break;
        }

        if (maxIter <= 0) {
          break;
        }
        maxIter--;
        // printf("yyy\n");
        h = h->twin()->next();} while( h != alpha->halfedge() );
      if (!found_apred) {
        printf("Failed to extend front due to malformed halfedges (1)\n");
        return false;
      }

      /*Compute b's successor*/
      HalfedgeIter b_succ = insideFront->twin()->next();

      o1->setNeighbors( o2,
                        i1,
                        alpha,
                        e1,
                        boundaryFace1);

      o2->setNeighbors( b_succ,
                        i2,
                        k,
                        e2,
                        boundaryFace2);

      i1->setNeighbors( insideFront->twin(),
                        o1,
                        k,
                        e1,
                        f);

      i2->setNeighbors( i1,
                        o2,
                        beta,
                        e2,
                        f);
      insideFront->twin()->setNeighbors( i2,
                                        insideFront,
                                        alpha,
                                        insideFront->edge(),
                                        f);
      a_pred->setNeighbors( o1,
                            a_pred->twin(),
                            a_pred->vertex(),
                            a_pred->edge(),
                            a_pred->face());



      // printf("Integrated a floating vertex\n");
      populate = f;
      return true;
    } else {
      printf("Adding an semi incorporated vertex\n");
      bool k_to_a = false;
      HalfedgeIter k_a;
      bool b_to_k = false;
      HalfedgeIter b_k;

      HalfedgeIter h = alpha->halfedge();
      int maxIter = 10;
      do {
          if (h->twin()->vertex() == k) {
            // printf("Glueing a to k\n");
            k_a = h->twin();
            k_a->edge()->BPisActive = false;
            k_to_a = true;
            break;
          }

          if (maxIter <= 0) {
            break;
          }
          maxIter--;
          printf("eee\n");
          h = h->twin()->next();} while( h != alpha->halfedge() );

      HalfedgeIter h_p = beta->halfedge();

      // printf("%p\n", h_p);
      int maxI = 15;
      do {
          if (h_p->twin()->vertex() == k) {
            printf("Glueing b to k\n");
            b_k = h_p;
            b_to_k = true;
            b_k->edge()->BPisActive = false;
            // b_k->edge()->BPisBoundary = false;
            break;
          }

          if (maxI <= 0) {
            break;
          }
          h_p = h_p->twin()->next();
          maxI--;
          printf("ttt\n");

          } while( h_p != beta->halfedge() );

      if ((k_to_a) && (b_to_k)) {

        /*Need to assign a face, and check that the outside is well shaped too*/
        FaceIter f = this->newFace();
        f->halfedge() = insideFront->twin();

        HalfedgeIter k_pred;
        bool found_kpred = false;
        HalfedgeIter h = k_a;
        maxIter = 15;
        do {
          if (h->twin()->next() == k_a) {
            k_pred = h->twin();
            found_kpred = true;
            break;
          }

          if (maxIter <= 0) {
            break;
          }
          maxIter--;
          printf("uuu\n");
          h = h->twin()->next();} while( h != k_a );
        if (!found_kpred) {
          printf("Failed to extend front due to malformed halfedges (1.5)\n");
          return false;
        } else if (k_pred != b_k) { /*need to fix the outsides to pass over instead of come inside*/
           k_pred->setNeighbors( b_k->next(),
                            k_pred->twin(),
                            k_pred->vertex(),
                            k_pred->edge(),
                            k_pred->face());
        }

        k_a->setNeighbors( insideFront->twin(),
                            k_a->twin(),
                            k,
                            k_a->edge(),
                            f);
        insideFront->twin()->setNeighbors( b_k,
                            insideFront,
                            alpha,
                            insideFront->edge(),
                            f);
        b_k->setNeighbors( k_a,
                            b_k->twin(),
                            beta,
                            b_k->edge(),
                            f);

        printf("Integrated a puzzle piece\n");
        populate = f;
        return true;
      } else if (k_to_a) {/*need to linearize k_a predecessor, 4 halfedges involved*/
        HalfedgeIter k_pred;
        bool found_kpred = false;
        HalfedgeIter h = k_a;
        int maxIter = 10;
        do {
          if (h->twin()->next() == k_a) {
            k_pred = h->twin();
            found_kpred = true;
            break;
          }
          if (maxIter <= 0) {
            break;
          }
          printf("fff\n");
          maxIter--;
          h = h->twin()->next();} while( h != k_a );
        if (!found_kpred) {
          printf("Failed to extend front due to malformed halfedges (2)\n");
          return false;
        } else {
          HalfedgeIter o2 = this->newHalfedge();
          HalfedgeIter i2 = this->newHalfedge();
          HalfedgeIter b_succ = insideFront->twin()->next();
          FaceIter f = this->newFace();
          f->halfedge() = i2;
          FaceIter boundaryFace1 = this->newBoundary();
          boundaryFace1->halfedge() = o2;
          EdgeIter e2 = this->newEdge();
          e2->BPisActive = true;
          e2->halfedge() = i2;
          beta->halfedge() = i2;
          k->halfedge() = k_a;

          o2->setNeighbors( b_succ,
                            i2,
                            k,
                            e2,
                            boundaryFace1);

          i2->setNeighbors( k_a,
                            o2,
                            beta,
                            e2,
                            f);

          insideFront->twin()->setNeighbors( i2,
                            insideFront,
                            alpha,
                            insideFront->edge(),
                            f);

          k_pred->setNeighbors( o2,
                            k_pred->twin(),
                            k_pred->vertex(),
                            k_pred->edge(),
                            k_pred->face());


          k_a->setNeighbors( insideFront->twin(),
                            k_a->twin(),
                            k,
                            k_a->edge(),
                            f);
          printf("Integrated a_to_k puzzle piece\n");
          populate = f;
          return true;
        }
      } else if (b_to_k) {/*need to linearize a_k predecessor, 4 halfedges involved*/
        /*Compute a's predecessor*/
        HalfedgeIter a_pred;
        bool found_apred = false;
        HalfedgeIter h = insideFront->twin();
        maxIter = 10;
        do {
          if (h->twin()->next() == insideFront->twin()) {
            a_pred = h->twin();
            found_apred = true;
            break;
          }


          if (maxIter <= 0) {
            break;
          }
          maxIter--;
          printf("ppp\n");
          h = h->twin()->next();} while( h != insideFront->twin() );
        if (!found_apred) {
          printf("Failed to extend front due to malformed halfedges (3)\n");
          return false;
        } else if (a_pred->twin()->vertex() != alpha) {
          printf("Failed to extend front due to malformed halfedges (3.5)\n");
          return false;
        }

        HalfedgeIter o1 = this->newHalfedge();
        HalfedgeIter i1 = this->newHalfedge();
        EdgeIter e1 = this->newEdge();
        e1->BPisActive = true;
        e1->halfedge() = i1;
        FaceIter f = this->newFace();
        f->halfedge() = i1;
        FaceIter boundaryFace1 = this->newBoundary();
        boundaryFace1->halfedge() = o1;
        k->halfedge() = i1;

        o1->setNeighbors( b_k->next(),
                            i1,
                            alpha,
                            e1,
                            boundaryFace1);

        i1->setNeighbors( insideFront->twin(),
                            o1,
                            k,
                            e1,
                            f);

        a_pred->setNeighbors( o1,
                            a_pred->twin(),
                            a_pred->vertex(),
                            a_pred->edge(),
                            a_pred->face());

        b_k->setNeighbors( i1,
                            b_k->twin(),
                            beta,
                            b_k->edge(),
                            f);

        b_k->twin()->setNeighbors( b_k->twin()->next(),
                            b_k,
                            k,
                            b_k->edge(),
                            b_k->twin()->face());

        printf("Integrated b_k inset face\n");
        populate = f;
        return true;
      } else { /*we are iceberging the wall that k is on, no need to mess with the outside edges yet*/

        /*Compute a's predecessor*/
        HalfedgeIter a_pred;
        bool found_apred = false;
        HalfedgeIter h = alpha->halfedge();
        do {
          if (h->twin()->next() == insideFront->twin()) {
            a_pred = h->twin();
            found_apred = true;
            break;
          }
          h = h->twin()->next();} while( h != alpha->halfedge() );
        if (!found_apred) {
          printf("Failed to extend front due to malformed halfedges (4)\n");
          return false;
        }

        HalfedgeIter k_pred_boundary;
        bool found_kpred_boundary = false;
         h = k->halfedge();
        do {
          if (h->isBoundary()) {
            k_pred_boundary = h;
            found_kpred_boundary = true;
            break;
          }
          printf("bccc\n");
          h = h->twin()->next();} while( h != k->halfedge() );
        if (!found_kpred_boundary) {
          printf("Failed to extend front due to malformed halfedges (4.5)\n");
          return false;
        }

        HalfedgeIter k_pred_boundary_enter;
        bool found_kpred_boundary_enter = false;
        HalfedgeIter h_enter = k->halfedge();
        maxIter = 10;
        do {
          if (h_enter->twin()->isBoundary()) {
            k_pred_boundary_enter = h_enter->twin();
            found_kpred_boundary_enter = true;
            break;
          }
          if (maxIter <= 0) {
            break;
          }

          maxIter--;
          printf("ieee\n");
          h_enter = h_enter->twin()->next();} while( h_enter != k->halfedge() );
        if (!found_kpred_boundary_enter) {
          printf("Failed to extend front due to malformed halfedges (4.6)\n");
          return false;
        }

        HalfedgeIter o1 = this->newHalfedge();
        HalfedgeIter i1 = this->newHalfedge();
        HalfedgeIter o2 = this->newHalfedge();
        HalfedgeIter i2 = this->newHalfedge();
        HalfedgeIter b_succ = insideFront->twin()->next();


        EdgeIter e1 = this->newEdge();
        e1->halfedge() = i1;
        e1->BPisActive = true;
        EdgeIter e2 = this->newEdge();
        e2->halfedge() = i2;
        e2->BPisActive = true;
        FaceIter f = this->newFace();
        f->halfedge() = i1;
        FaceIter boundaryFace1 = this->newBoundary();
        boundaryFace1->halfedge() = o1;
        FaceIter boundaryFace2 = this->newBoundary();
        boundaryFace2->halfedge() = o2;




        k_pred_boundary_enter->setNeighbors( o2,
                                    k_pred_boundary_enter->twin(),
                                    k_pred_boundary_enter->vertex(),
                                    k_pred_boundary_enter->edge(),
                                    k_pred_boundary_enter->face());



        i1->setNeighbors( insideFront->twin(),
                          o1,
                          k,
                          e1,
                          f);

        i2->setNeighbors( i1,
                          o2,
                          beta,
                          e2,
                          f);

        insideFront->twin()->setNeighbors( i2,
                          insideFront,
                          alpha,
                          insideFront->edge(),
                          f);

        o1->setNeighbors( k_pred_boundary,
                          i1,
                          alpha,
                          e1,
                          boundaryFace1);

        o2->setNeighbors( b_succ,
                          i2,
                          k,
                          e2,
                          boundaryFace2);

        a_pred->setNeighbors( o1,
                  a_pred->twin(),
                  a_pred->vertex(),
                  a_pred->edge(),
                  a_pred->face());

        k->halfedge() = i1;
        printf("Integrated an iceberg bridge\n");
        populate = f;
        return true;
      }
      printf("Failed to integrate anything?\n");

    }
    return false;
  }

  bool computeSeedTriangleGivenPoint( VertexIter sigma, std::vector<VertexIter> accel_struct, double rho, VertexIter& populate_alpha, VertexIter& populate_beta) {
    if (accel_struct.size() < 2) {
      printf("computeSeedTriangleGivenPoint: Cannot compute seed triangle with less than 2 free vertices\n");
      return false;
    }
    for (VertexIter sigma_alpha : accel_struct) {

      if (sigma_alpha->BPisUsed) {
            continue;
          }

      if (sigma_alpha->position.x != sigma->position.x ||
          sigma_alpha->position.y != sigma->position.y ||
          sigma_alpha->position.z != sigma->position.z) { /*We cannot use sigma as two points on the triangle*/

        for (VertexIter sigma_beta : accel_struct) {
          if (sigma_beta->BPisUsed) {
            continue;
          }

          if ((sigma_beta->position.x != sigma->position.x || sigma_beta->position.y != sigma->position.y || sigma_beta->position.z != sigma->position.z)
            && (sigma_beta->position.x != sigma_alpha->position.x || sigma_beta->position.y != sigma_alpha->position.y || sigma_beta->position.z != sigma_alpha->position.z)) { /*We cannot use sigma or sigma_alpha as two points on the triangle*/

            Vector3D ball_center;
            Vector3D normal_orientation;
            bool orientable = false;
            bool flipped = false;
            if (circumsphere_center(sigma->position , sigma_alpha->position, sigma_beta->position, rho, ball_center)) { /*A sphere can be computed from these three*/
              // printf("A sphere can be placed on the found vertices\n");
              normal_orientation = sigma->norm + sigma_alpha->norm + sigma_beta->norm;
              if (dot(normal_orientation, ball_center - ((sigma->position + sigma_alpha->position + sigma_beta->position) * 1.0/3.0)) > 0.0)  {
                // printf("Orientable originally\n");
                orientable = true;
              } else if (circumsphere_center(sigma->position , sigma_beta->position, sigma_alpha->position, rho, ball_center)) {
                // printf("Orientable when flipped\n");
              /*The normal of the computed ball and triangle did not orient with the vertex normals, compute the counterpart ball isntead*/
                flipped = true;
                orientable = true;
              } else {
                orientable = false;
              }
            }

            if (orientable) {/*We proceed with the ball center populated, and compute if this is a valid configuration*/
              // printf("And they are orientable\n");

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
                if (flipped) {
                  populate_alpha = sigma_beta;
                  populate_beta = sigma_alpha;
                }
                return true;
              }
            }
          }
        }
      }
    }
    return false;
  }





    std::vector<VertexIter> MeshResampler::ball_pivot( HalfedgeMesh& mesh, HalfedgeIter& populate, std::vector<FaceIter>& current_faces, int max_count) {

    set_rho(mesh, 0.3, false);
    printf("Beginning preprocessing...");
    Vector3D centroid = Vector3D();
    double n = 0.0;
    for( VertexIter e = mesh.verticesBegin(); e != mesh.verticesEnd(); e++ ) {
      centroid+=e->position;
      n+=1.0;
    }
    centroid/=n;

    for (const auto &entry : map) {
      // printf("Entry!\n");
      std::vector<VertexIter> neighbors = get_neighbors( ((*entry.second)[0])->position );

      for (VertexIter v : *entry.second) {
          v->norm = v->position - centroid;
        // } else {
        //   printf("Normal creation failed, trying again\n");
        //   std::vector<VertexIter> v;
        //   return v;
        // }
      }
    }

    for (const auto &entry : cluster_vertices(mesh, 3.0)) {
      Vector3D local_centroid = Vector3D();
      double local_n = 0.0;
      for( VertexIter v : (*entry.second)) {
        local_n+=1.0;
        local_centroid += v->position;
      }
      local_centroid/=local_n;
      for( VertexIter v : (*entry.second)) {
        v->norm += 1.5*(v->position - local_centroid) ;
        // printf("vnorm is %4f %4f %4f\n", v->norm.x, v->norm.y, v->norm.z);
         v->norm = v->norm.unit() ;
         v->norm/=50.0;
      }
      make_normals_consistent((*entry.second));
    }

    printf("Done \n");

    //TODO clear everything!
    printf("Deleting existing topology...");
    std::vector<EdgeIter> b1;
    for( EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++ ) {
      b1.push_back(e);
    }
    for( EdgeIter e : b1 ) {
      mesh.deleteEdge(e);
    }
    std::vector<HalfedgeIter> b2;
    for( HalfedgeIter e = mesh.halfedgesBegin(); e != mesh.halfedgesBegin(); e++ ) {
      b2.push_back(e);
    }
    for( HalfedgeIter e : b2 ) {
      mesh.deleteHalfedge(e);
    }
    std::vector<FaceIter> b3;
    for( FaceIter e = mesh.facesBegin(); e != mesh.facesEnd(); e++ ) {
      b3.push_back(e);
    }
    for( FaceIter e : b3 ) {
      mesh.deleteFace(e);
    }

    printf("Done\n");


    //TODO compute the radius list we want by doing some statistics on the points
    //TODO build a mesh acceleration structure (voxel grid), so that we can look at a vertices nearest neighbors
    //TODO Iterating from smallest rho to largest
      std::deque<VertexIter> unused_vertices;
      std::vector<VertexIter> frozen_vertices;
      std::deque<EdgeIter> active_edges;
      std::deque<EdgeIter> front_edges;

      //TODO create a queue of free verticesmax_count
      for( VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++ ) {
        v->BPisUsed = false;
        unused_vertices.push_back(v);
      }

      while (true) {
        printf("Doing main loop\n");
        //TODO Loop on compute a seed triangle
        VertexIter candidate_sigma;
        bool seedFound = false;
        VertexIter sigma_alpha;
        VertexIter sigma_beta;
        while (!(seedFound) && unused_vertices.size() > 0) {

          // printf("Doing seed find loop\n");;
          bool candidate_found = false;
          /*Use voxel accel struct to get the closest points here*/
          while (unused_vertices.size() > 0) {
            candidate_sigma = unused_vertices.front();
            // printf("Candidate suggested\n");
            unused_vertices.pop_front();
            if ((candidate_sigma->BPisUsed)) {
              // printf("But continued\n");
              continue;
            } else {
              candidate_found = true;
              break;
            }
          }

          if (candidate_found) {
            // printf("Candidate_found\n");
            seedFound = computeSeedTriangleGivenPoint(candidate_sigma, get_neighbors(candidate_sigma->position), rho, sigma_alpha, sigma_beta);
            if (!(seedFound)) {
              printf("Froze a vertex\n");
              frozen_vertices.push_back(candidate_sigma);
            }
          }
        }


        if (seedFound) {
          printf("Seed triangle point found, assimilating it to the mesh we have now\n");
          mesh.createSeedTriangle(candidate_sigma, sigma_alpha, sigma_beta);
          EdgeIter e1 = candidate_sigma->halfedge()->edge();
          current_faces.push_back(candidate_sigma->halfedge()->face());
          candidate_sigma->halfedge()->face()->isSeed = true;
          EdgeIter e2 = sigma_alpha->halfedge()->edge();
          EdgeIter e3 = sigma_beta->halfedge()->edge();
          active_edges.push_back(e1);
          active_edges.push_back(e2);
          active_edges.push_back(e3);
          e1->BPisActive = true;
          e1->BPisBoundary = false;
          e2->BPisActive = true;
          e2->BPisBoundary = false;
          e3->BPisActive = true;
          e3->BPisBoundary = false;
          candidate_sigma->BPisUsed = true;
          sigma_alpha->BPisUsed = true;
          sigma_beta->BPisUsed = true;

          // printf("Candidate Sigma is here: %4f %4f %4f \n", candidate_sigma->position.x, candidate_sigma->position.y, candidate_sigma->position.z);
          // printf("returned correctly\n");
        } else {
          /*We must try another rho or kill the floating vertices and call it done*/
          printf("No more candidate seeds found %d\n", frozen_vertices.size());
          return frozen_vertices ;
        }



        //TODO compute the "active" edge e, or edge on the fringe that we must pivot over
        /*Use voxel accel struct to get the closest points here*/
        int iterations = 1;
        int iterCount = 0;
        while (active_edges.size() > 0) {

          bool active_edge_found = false;
          EdgeIter candidate_active_edge;

          while (active_edges.size() > 0) {
            candidate_active_edge = active_edges.front();
            printf("Active Edge Candidate suggested\n");
            active_edges.pop_front();
            if ((candidate_active_edge->BPisActive)) {
              active_edge_found = true;
              candidate_active_edge->BPisActive = false;
              break;
            } else {
              printf("But continued\n");
            }
          }


          if (active_edge_found) {
              // printf("Active Edge Candidate_found\n");

            //TODO not_used(), not_internal
            // 3. if (Vertex k = pivot(e) && ( not_used(k) || not_internal(k) ) )
              VertexIter k;
              if (pivot_from( candidate_active_edge->halfedge(), rho, k, this)) {


                printf("BPA: Pivoted to populate k, now going to check receiving point validity!\n");

                if (!(k->BPisUsed) || (k->isBoundary())) {
                  printf("BPA: Validated vertex k to incorporate!\n");
                  //TODO, function that reassigns half edge pointers, edge pointers, face pointers, to make a triangle
                  // 4. output triangle(i,  k , j )
                  HalfedgeIter insideFront = candidate_active_edge->halfedge();

                  FaceIter f;
                  if (mesh.createFrontTriangle(insideFront, k, f)) {
                    printf("Triangle integrated, testing the edges\n");
                    EdgeIter e1 = insideFront->twin()->next()->edge();
                    current_faces.push_back(f);
                    insideFront->twin()->face()->isSeed = false;
                    EdgeIter e2 = insideFront->twin()->next()->next()->edge();
                    printf("Edges did not segmentation fault\n");

                    if (e1->BPisActive) {
                      active_edges.push_back(e1);
                    }

                    if (e2->BPisActive) {
                      active_edges.push_back(e2);
                    }

                    k->BPisUsed = true;

                    printf("Candidate Sigma is here: %4f %4f %4f \n", candidate_sigma->position.x, candidate_sigma->position.y, candidate_sigma->position.z);
                    if (iterCount >= max_count) {
                      for( FaceIter e = mesh.facesBegin(); e != mesh.facesEnd(); e++ ) {
                        // printf("testing\n");
                        e->normal();
                        // printf("okay\n");
                      }
                      printf("Premature debug termination\n");
                      printf("%d\n", iterCount);
                      return frozen_vertices;
                    }
                    iterCount++;
                    printf("iteration %d\n", iterCount);

                  } else {
                    printf("Ball iteration terminated in a bad spot %d.\n", iterCount);
                    // populate = insideFront;
                    continue;
                    // return frozen_vertices ;
                  }

                } else {
                  // printf("The point was invalid, marking as boundary\n");
                //TODO join(e, ek1, ek2), which takes in an old edge, and two new edges, marks the old as internal and the new as FRONT, luckily for us, we do not need to worry about glueing
                  //here because a vertex will just overwrite its edge pointers, and half edges are expected to be opposite facing
                // 5. join(e(i,j) , k , F)
            // 8 . else
            // TODO function mark an edge as a fixed boundary
            // 9 . mark as boundary(e(i;j))
                printf("Marked the edge as boundary due to resulting mesh not being manifold\n");
                candidate_active_edge->BPisBoundary = true;
                front_edges.push_back(candidate_active_edge);
              }
            } else {
                printf("Marked the edge as boundary due to ball making it around to other side\n");
                candidate_active_edge->BPisBoundary = true;
                front_edges.push_back(candidate_active_edge);
            }
          }
        }
        // if (active_edges.size() == 0) {
        //   iterations++;
        //   set_rho(mesh, rho*75, true);
        //   for( VertexIter v : unused_vertices ) {
        //     if (v->BPisUsed = false) {
        //
        //     }
        //   }
        // }

    }
    printf("performing sanity checks\n");

    printf("Vertices\n");
    for( VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++ ) {
        printf("%4f\n",v->position.x);

    }

    printf("done\n");
    printf("BPA all done\n");
    return frozen_vertices;

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
