#include "sphere.h"

#include <cmath>

#include  "../bsdf.h"
#include "../misc/sphere_drawing.h"

namespace CGL { namespace StaticScene {

bool Sphere::test(const Ray& r, double& t1, double& t2) const {

  // Part 1, Task 4:
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.

  double a = dot(r.d, r.d);
  double b = dot(2.0*(r.o - o), r.d);
  double c = dot(r.o - o, r.o - o) - r2;
  double discriminant = sqrt(pow(b,2) - 4*a*c);
  double t_1 = (-b + discriminant)/(2*a);
  double t_2 = (-b - discriminant)/(2*a);

  if (t_1 < t_2) {
    if ((t_1 > r.min_t && t_1 < r.max_t)) {
      t1 = t_1;
      t2 = t_2;
      return true;
    } else if ((t_2 > r.min_t && t_2 < r.max_t)) {
      t1 = t_2;
      t2 = t_1;
      return true;
    } else {
      return false;
    }
  } else {
    if ((t_2 > r.min_t && t_2 < r.max_t)) {
      t1 = t_2;
      t2 = t_1;
      return true;
    } else if ((t_1 > r.min_t && t_1 < r.max_t)) {
      t1 = t_1;
      t2 = t_2;
      return true;
    } else {
      return false;
    }
  }
}

bool Sphere::intersect(const Ray& r) const {

  // Part 1, Task 4:
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
  double t1; 
  double t2; 
  if (test(r, t1, t2)) {
    return true;
  }
  return false;

}

bool Sphere::intersect(const Ray& r, Intersection *i) const {

  // Part 1, Task 4:
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.

  double t1; 
  double t2; 
  if (test(r, t1, t2)) {
    i->t = t1;
    i->n = ((r.o+t1*r.d)-o)/this->r;
    i->primitive = this;
    i->bsdf = get_bsdf();
    if (t1 < r.max_t) {
      r.max_t = t1;
    }
    return true;
  }
  return false;

}

void Sphere::draw(const Color& c) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color& c) const {
    //Misc::draw_sphere_opengl(o, r, c);
}


} // namespace StaticScene
} // namespace CGL
