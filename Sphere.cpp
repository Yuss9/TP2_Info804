/**
@file Sphere.cpp
*/
#include <cmath>
#include "Sphere.h"

void rt::Sphere::draw(Viewer & /* viewer */)
{
  Material m = material;
  // Taking care of south pole
  glBegin(GL_TRIANGLE_FAN);
  glColor4fv(m.ambient);
  glMaterialfv(GL_FRONT, GL_DIFFUSE, m.diffuse);
  glMaterialfv(GL_FRONT, GL_SPECULAR, m.specular);
  glMaterialf(GL_FRONT, GL_SHININESS, m.shinyness);
  Point3 south_pole = localize(-90, 0);
  glNormal3fv(getNormal(south_pole));
  glVertex3fv(south_pole);
  for (int x = 0; x <= NLON; ++x)
  {
    Point3 p = localize(-90 + 180 / NLAT, x * 360 / NLON);
    glNormal3fv(getNormal(p));
    glVertex3fv(p);
  }
  glEnd();
  // Taking care of in-between poles
  for (int y = 1; y < NLAT - 1; ++y)
  {
    glBegin(GL_QUAD_STRIP);
    glColor4fv(m.ambient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, m.diffuse);
    glMaterialfv(GL_FRONT, GL_SPECULAR, m.specular);
    glMaterialf(GL_FRONT, GL_SHININESS, m.shinyness);
    for (int x = 0; x <= NLON; ++x)
    {
      Point3 p = localize(-90 + y * 180 / NLAT, x * 360 / NLON);
      Point3 q = localize(-90 + (y + 1) * 180 / NLAT, x * 360 / NLON);
      glNormal3fv(getNormal(p));
      glVertex3fv(p);
      glNormal3fv(getNormal(q));
      glVertex3fv(q);
    }
    glEnd();
  }
  // Taking care of north pole
  glBegin(GL_TRIANGLE_FAN);
  glColor4fv(m.ambient);
  glMaterialfv(GL_FRONT, GL_DIFFUSE, m.diffuse);
  glMaterialfv(GL_FRONT, GL_SPECULAR, m.specular);
  glMaterialf(GL_FRONT, GL_SHININESS, m.shinyness);
  Point3 north_pole = localize(90, 0);
  glNormal3fv(getNormal(north_pole));
  glVertex3fv(north_pole);
  for (int x = NLON; x >= 0; --x)
  {
    Point3 p = localize(-90 + (NLAT - 1) * 180 / NLAT, x * 360 / NLON);
    glNormal3fv(getNormal(p));
    glVertex3fv(p);
  }
  glEnd();
}

rt::Point3
rt::Sphere::localize(Real latitude, Real longitude) const
{
  static const Real conv_deg_rad = 2.0 * M_PI / 360.0;
  latitude *= conv_deg_rad;
  longitude *= conv_deg_rad;
  return center + radius * Point3(cos(longitude) * cos(latitude),
                                  sin(longitude) * cos(latitude),
                                  sin(latitude));
}

rt::Vector3
rt::Sphere::getNormal(Point3 p)
{
  Vector3 u = p - center;
  Real l2 = u.dot(u);
  if (l2 != 0.0)
    u /= sqrt(l2);
  return u;
}

rt::Material
rt::Sphere::getMaterial(Point3 /* p */)
{
  return material; // the material is constant along the sphere.
}

/*

Il calcule la distance entre le point d'origine du rayon et le cercle, puis utilise le théorème de Pythagore pour déterminer si le rayon intersecte ou non le cercle. Si le rayon intersecte le cercle, le code calcule les points d'intersection et renvoie -1.0f pour indiquer que l'intersection a eu lieu. Sinon, il renvoie 1.0f pour indiquer qu'il n'y a pas d'intersection.

*/
rt::Real
rt::Sphere::rayIntersection(const Ray &ray, Point3 &p)
{
  // TO DO
  Vector3 OP = this->center - ray.origin;
  Vector3 w = ray.direction / ray.direction.norm();

  Real distanceOH = w.dot(OP);
  Real distanceHPCarre = OP.dot(OP) - (distanceOH * distanceOH);

  if (distanceHPCarre > (this->radius * this->radius))
  {
    return 1.0f;
  }
  else
  {
    Real b = sqrt(this->radius * this->radius - distanceHPCarre);
    Real t1 = (distanceOH - b);
    Real t2 = (distanceOH + b);

    if (t1 < 0 && t2 < 0)
      return 1.0f;
    else if (t1 > 0 && t2 > 0)
    {
      p = ray.origin + std::min(t1, t2) * w;
      return -1.0f;
    }
    else
    {
      if (t1 < 0)
      {
        p = ray.origin + t2 * w;
      }
      else
      {
        p = ray.origin + t1 * w;
      }
      return -1.0f;
    }
  }
}
