#include "GraphicalObject.h"

/// Namespace RayTracer
namespace rt
{

    struct WaterPlane : public GraphicalObject
    {
    
        WaterPlane(Point3 c, Vector3 u, Vector3 v,
                   Material main_m, Material band_m, Real w) : c{c}, u{u}, v{v}, main_m{main_m}, band_m{band_m}, w{w} {};

        void coordinates(Point3 p, Real &x, Real &y)
        {
            x = u[0] + p[0];
            y = u[1] + p[1];
        }

        void init(Viewer & /* viewer */) {}

        void draw(Viewer &viewer) override
        {
            glBegin(GL_TRIANGLES);
            glColor4fv(main_m.ambient);
            glMaterialfv(GL_FRONT, GL_DIFFUSE, main_m.diffuse);
            glMaterialfv(GL_FRONT, GL_SPECULAR, main_m.specular);
            glMaterialf(GL_FRONT, GL_SHININESS, main_m.shinyness);
            glVertex3f(0, 0, 0.0);
            glVertex3f(0, 1000.0, 0.0);
            glVertex3f(1000.0, 0, 0.0);
            glEnd();

            glBegin(GL_TRIANGLES);
            glColor4fv(main_m.ambient);
            glMaterialfv(GL_FRONT, GL_DIFFUSE, main_m.diffuse);
            glMaterialfv(GL_FRONT, GL_SPECULAR, main_m.specular);
            glMaterialf(GL_FRONT, GL_SHININESS, main_m.shinyness);
            glVertex3f(0, 1000.0, 0.0);
            glVertex3f(1000.0, 1000.0, 0.0);
            glVertex3f(1000.0, 0, 0.0);

            glEnd();
        }

        Vector3 getNormal(Point3 p)
        {
            return p + u.cross(v);
        }

        Material getMaterial(Point3 p)
        {
            Real x, y;
            coordinates(p, x, y);
            return ((int)x == (int)(x + w)) && ((int)y == (int)(y + w)) ? main_m : band_m;
        }

        Real rayIntersection(const Ray &ray, Point3 &p)
        {
            Real t;
            Vector3 N = this->getNormal(c) / this->getNormal(c).norm();
            Vector3 W = ray.direction / ray.direction.norm();
            Real wn = W.dot(N);
            Vector3 tmp = (c - ray.origin) / (c - ray.origin).norm();
            if (wn == 0.0f)
            {

                if ((N.dot(tmp) / wn) == 0)
                {
                    t = N.dot(tmp) / wn;
                    p = ray.origin + W * t;
                    if (t >= 0.0f)
                    {
                        // std::cout << p << std::endl;
                        return -1.0f;
                    }
                }
                else
                {
                    return 1.0f;
                }
            }
            else
            {
                t = N.dot(tmp) / wn;
                p = ray.origin + W * t;
                if (t >= 0.0f)
                {
                    // std::cout << p << std::endl;
                    return -1.0f;
                }
            }
            return 1.0f;
        }

    public:
        Point3 c;
        Vector3 u, v;
        Material band_m, main_m;
        Real w;
    };
}