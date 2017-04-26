#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

  // Part 2, Task 2:
  // Implement ray - bounding box intersection test
  // If the ray intersected the bouding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.
  /*x axis*/
  double tx1 = (min.x - r.o.x)/ (r.d.x);
  double tx2 = (max.x - r.o.x)/ (r.d.x);
  double ty1 = (min.y - r.o.y)/ (r.d.y);
  double ty2 = (max.y - r.o.y)/ (r.d.y);
  double tz1 = (min.z - r.o.z)/ (r.d.z);
  double tz2 = (max.z - r.o.z)/ (r.d.z);

  double tmpmin = std::max( std::min(tx1, tx2), std::max( std::min(ty1, ty2), std::min(tz1, tz2)));
  double tmpmax = std::min( std::max(tx1, tx2), std::min( std::max(ty1, ty2), std::max(tz1, tz2)));

  if (tmpmax < tmpmin) {
    return false;
  } else {
    if (tmpmax < r.min_t || tmpmin > r.max_t) {
      return false;
    } else {
      return true;
    }
  }
}

void BBox::draw(Color c) const {

  glColor4f(c.r, c.g, c.b, c.a);

	// top
	glBegin(GL_LINE_STRIP);
	glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
	glEnd();

	// bottom
	glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
	glEnd();

	// side
	glBegin(GL_LINES);
	glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
	glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
	glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
	glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
	glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL
