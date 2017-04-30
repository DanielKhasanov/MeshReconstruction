#include "student_code.h"
#include "mutablePriorityQueue.h"

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
    center = c + cross(o - i, i - j).unit()*x_mag;
    return true;
  }

  //TODO pivot() function that takes two vertices, a mesh acceleration structure, and a radius rho 
    //and computes the vertex which when touched will make ensure no other vertices are in the ball
    //Cannot fail on the inside half edge, as it is a well formed triangle via our seed or algorithm prior.
  void pivot_from( HalfedgeIter inside_halfedge, double rho) {


    Vector3D sigma_i = inside_halfedge->vertex()->position;
    Vector3D sigma_j = inside_halfedge->next->vertex()->position;
    Vector3D m = 0.5*(sigma_i + sigma_j);
    Vector3D sigma_o = inside_halfedge->next->next->vertex()->position;
    Vector3D c_ijo = circumsphere_center(sigma_i, sigma_j, sigma_o, rho);

    std::vector<Vertex*> rho_closest;
    //rho_closest = accel_struct->f(m , 2*rho);
    for (Vertex &v : rho_closest) {
      if (v.position != sigma_o) {
        Vector3D cx = new Vector3D();
        if (circumsphere_center(sigma_i, sigma_j, v.position, rho, *cx)) {

          double num_touching = 0;
          for (Vertex &v : rho_closest) {
            int dist_from_center = (v.position - cx).norm();
            if (dist_from_center < rho) {
              break;
            } else if (dist_from_center == rho) {
              num_touching += 1;
            }
          }

        } else {

        ;

      }
    }


  }

  void MeshResampler::ball_pivot( HalfedgeMesh& mesh) {

    //TODO compute the radius list we want by doing some statistics on the points
    //TODO build a mesh acceleration structure (voxel grid), so that we can look at a vertices nearest neighbors

    //TODO Iterating from smallest rho to largest

      //TODO Loop on compute a seed triangle

        //TODO compute the "active" edge e, or edge on the fringe that we must pivot over

        //TODO not_used(), not_internal
        // 3. if (Vertex k = pivot(e) && ( not_used(k) || not_internal(k) ) )
          //TODO, function that reassigns half edge pointers, edge pointers, face pointers, to make a triangle
          // 4. output triangle(i,  k , j )

          //TODO join(e, ek1, ek2), which takes in an old edge, and two new edges, marks the old as internal and the new as FRONT, luckily for us, we do not need to worry about glueing 
            //here because a vertex will just overwrite its edge pointers, and half edges are expected to be opposite facing
          // 5. join(e(i,j) , k , F)

        // 8 . else

          // TODO function mark an edge as a fixed boundary 
          // 9 . mark as boundary(e(i;j) )

    Vector3D i = Vector3D(123, 456 , 789);
    Vector3D j = Vector3D(666,66,6);
    Vector3D o = Vector3D(34, 43.24, -3422.0321);
    Vector3D c = circumsphere_center( i, j, o, 234556);

    printf("Circumsphere center is : %4f %4f %4f\n", c.x, c.y, c.z);
    printf("Distance from i is : %4f\n", (i-c).norm());
    printf("Distance from j is : %4f\n", (j-c).norm());
    printf("Distance from o is : %4f\n", (o-c).norm());

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
