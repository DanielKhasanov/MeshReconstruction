#include "bvh.h"

#include "CGL/CGL.h"
#include "static_scene/triangle.h"

#include <iostream>
#include <stack>


using namespace std;

namespace CGL { namespace StaticScene {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  root = construct_bvh(_primitives, max_leaf_size);

}

BVHAccel::~BVHAccel() {
  if (root) delete root;
}

BBox BVHAccel::get_bbox() const {
  return root->bb;
}

void BVHAccel::draw(BVHNode *node, const Color& c) const {
  if (node->isLeaf()) {
    for (Primitive *p : *(node->prims))
      p->draw(c);
  } else {
    draw(node->l, c);
    draw(node->r, c);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color& c) const {
  if (node->isLeaf()) {
    for (Primitive *p : *(node->prims))
      p->drawOutline(c);
  } else {
    drawOutline(node->l, c);
    drawOutline(node->r, c);
  }
}

typedef struct Bucket {
  int prim_count;
  BBox bbox;
  double ceiling;
} Bucket;

double heuristic(int pca, int pcb, BBox bba, BBox bbb) {
  return (1.0/(pow(( (double) pca)*( (double) pcb), 2 ))) * (1.0/(((double) pca / bba.surface_area()) + ( ((double) pcb) / bbb.surface_area())));

}

BVHNode *BVHAccel::construct_bvh(const std::vector<Primitive*>& prims, size_t max_leaf_size) {
  
  // Part 2, Task 1:
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.

  BBox bbox;
  for (Primitive *p : prims) {
    BBox bb = p->get_bbox();
    bbox.expand(bb);
  }

  BVHNode *node = new BVHNode(bbox);
  node->prims = new vector<Primitive *>(prims);
  if (prims.size() < max_leaf_size ) {
    
    return node;
  } else {
    int axis = 2;
    if (bbox.extent.x > bbox.extent.y) {
      if (bbox.extent.x > bbox.extent.z) {
        axis = 0;
      } 
    } else {
      if (bbox.extent.y > bbox.extent.z) {
        axis = 1;
      } 
    }

    std::sort(node->prims->begin() , node->prims->end(), [&axis](Primitive* a, Primitive* b) { return a->get_bbox().centroid()[axis] < b->get_bbox().centroid()[axis]; });

  
    // std::vector<Primitive *> l =  new vector<Primitive *>(node->prims->begin(), node->prims->begin() + prims.size()/2);
    // std::vector<Primitive *> r =  new vector<Primitive *>(node->prims->begin() + prims.size()/2, node->prims->end());

    // int newSplit = (prims.size())/2;
    // int i = 0;
    // for (Primitive *p : node->prims ) {
    //   if (i < newSplit) {
    //     l.push_back(p);
    //   } else {
    //     r.push_back(p);
    //   }
    //   i++;
    // }

 
      node->l = construct_bvh(  vector<Primitive *>(node->prims->begin(), node->prims->begin() + prims.size()/2), max_leaf_size );
      node->r = construct_bvh(  vector<Primitive *>(node->prims->begin() + prims.size()/2, node->prims->end()), max_leaf_size);
      return node;
    
  }
}


bool BVHAccel::intersect(const Ray& ray, BVHNode *node) const {
  // Part 2, task 3: replace this.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.

  bool hit = false;

  if (node->bb.intersect(ray, ray.min_t , ray.max_t )) {
    if (node->isLeaf()) {
      
        for (Primitive *p : *(node->prims)) {
          if (p->intersect(ray)) {
              hit = true;
          } 
        }
      } else {
        bool hit1 =  (intersect(ray, node->l));
        bool hit2 = (intersect(ray, node->r));
        hit = (hit1 || hit2);
      }
  }
  return hit;

}

bool BVHAccel::intersect(const Ray& ray, Intersection* i, BVHNode *node) const {
  // Part 2, task 3: replace this

  bool hit  = false;

  if (node->bb.intersect(ray, ray.min_t , ray.max_t )) {
    if (node->isLeaf()) {
        for (Primitive *p : *(node->prims)) {
          if (p->intersect(ray, i)) {
              hit = true;
          } 
        }
      } else {
        bool hit1 =  intersect(ray, i, node->l);
        bool hit2 = intersect(ray, i, node->r);
        hit = hit1 || hit2;
      }
  }
  return hit;

}

}  // namespace StaticScene
}  // namespace CGL
