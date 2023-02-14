/**
@file Renderer.h
@author JOL
*/
#pragma once
#ifndef _RENDERER_H_
#define _RENDERER_H_

#include "Color.h"
#include "Image2D.h"
#include "Ray.h"

/// Namespace RayTracer
namespace rt
{

  struct Background
  {
    virtual Color backgroundColor(const Ray &ray) = 0;
  };
  struct MyBackground : public Background
  {
    Color backgroundColor(const Ray &ray)
    {
      Color result;
      float z = ray.direction.at(2);
      if (z >= 0.0f && z <= 0.5f)
      {
        return Color(1 - 2 * z, 1 - 2 * z, 1);
      }
      if (z > 0.5f && z <= 1.0f)
      {
        return Color(0, 0, 1.0f - (z - 0.5f) * 2);
      }
      else
      {

        Real x = -0.5f * ray.direction[0] / ray.direction[2];
        Real y = -0.5f * ray.direction[1] / ray.direction[2];
        Real d = sqrt(x * x + y * y);
        Real t = std::min(d, 30.0f) / 30.0f;
        x -= floor(x);
        y -= floor(y);
        if (((x >= 0.5f) && (y >= 0.5f)) || ((x < 0.5f) && (y < 0.5f)))
          result += (1.0f - t) * Color(0.2f, 0.2f, 0.2f) + t * Color(1.0f, 1.0f, 1.0f);
        else
          result += (1.0f - t) * Color(0.4f, 0.4f, 0.4f) + t * Color(1.0f, 1.0f, 1.0f);
        return result;
      }
    }
  };

  inline void progressBar(std::ostream &output,
                          const double currentValue, const double maximumValue)
  {
    static const int PROGRESSBARWIDTH = 60;
    static int myProgressBarRotation = 0;
    static int myProgressBarCurrent = 0;
    // how wide you want the progress meter to be
    double fraction = currentValue / maximumValue;

    // part of the progressmeter that's already "full"
    int dotz = static_cast<int>(floor(fraction * PROGRESSBARWIDTH));
    if (dotz > PROGRESSBARWIDTH)
      dotz = PROGRESSBARWIDTH;

    // if the fullness hasn't changed skip display
    if (dotz == myProgressBarCurrent)
      return;
    myProgressBarCurrent = dotz;
    myProgressBarRotation++;

    // create the "meter"
    int ii = 0;
    output << "[";
    // part  that's full already
    for (; ii < dotz; ii++)
      output << "#";
    // remaining part (spaces)
    for (; ii < PROGRESSBARWIDTH; ii++)
      output << " ";
    static const char *rotation_string = "|\\-/";
    myProgressBarRotation %= 4;
    output << "] " << rotation_string[myProgressBarRotation]
           << " " << (int)(fraction * 100) << "/100\r";
    output.flush();
  }

  /// This structure takes care of rendering a scene.
  struct Renderer
  {

    /// The scene to render
    Scene *ptrScene;
    /// The origin of the camera in space.
    Point3 myOrigin;
    /// (myOrigin, myOrigin+myDirUL) forms a ray going through the upper-left
    /// corner pixel of the viewport, i.e. pixel (0,0)
    Vector3 myDirUL;
    /// (myOrigin, myOrigin+myDirUR) forms a ray going through the upper-right
    /// corner pixel of the viewport, i.e. pixel (width,0)
    Vector3 myDirUR;
    /// (myOrigin, myOrigin+myDirLL) forms a ray going through the lower-left
    /// corner pixel of the viewport, i.e. pixel (0,height)
    Vector3 myDirLL;
    /// (myOrigin, myOrigin+myDirLR) forms a ray going through the lower-right
    /// corner pixel of the viewport, i.e. pixel (width,height)
    Vector3 myDirLR;
    Background *ptrBackground;
    int myWidth;
    int myHeight;

    Renderer() : ptrScene(0) {}
    Renderer(Scene &scene) : ptrScene(&scene)
    {
      ptrBackground = new MyBackground();
    }
    void setScene(rt::Scene &aScene) { ptrScene = &aScene; }

    void setViewBox(Point3 origin,
                    Vector3 dirUL, Vector3 dirUR, Vector3 dirLL, Vector3 dirLR)
    {
      myOrigin = origin;
      myDirUL = dirUL;
      myDirUR = dirUR;
      myDirLL = dirLL;
      myDirLR = dirLR;
    }

    void setResolution(int width, int height)
    {
      myWidth = width;
      myHeight = height;
    }

    /// The main rendering routine
    void render(Image2D<Color> &image, int max_depth)
    {
      std::cout << "Rendering into image ... might take a while." << std::endl;
      image = Image2D<Color>(myWidth, myHeight);
      for (int y = 0; y < myHeight; ++y)
      {
        Real ty = (Real)y / (Real)(myHeight - 1);
        progressBar(std::cout, ty, 1.0);
        Vector3 dirL = (1.0f - ty) * myDirUL + ty * myDirLL;
        Vector3 dirR = (1.0f - ty) * myDirUR + ty * myDirLR;
        dirL /= dirL.norm();
        dirR /= dirR.norm();
        for (int x = 0; x < myWidth; ++x)
        {
          Real tx = (Real)x / (Real)(myWidth - 1);
          Vector3 dir = (1.0f - tx) * dirL + tx * dirR;
          Ray eye_ray = Ray(myOrigin, dir, max_depth);
          Color result = trace(eye_ray);
          image.at(x, y) = result.clamp();
        }
      }
      std::cout << "Done." << std::endl;
    }

    /// The rendering routine for one ray.
    /// @return the color for the given ray.
    Color trace(const Ray &ray)
    {
      assert(ptrScene != 0);
      // Color result = Color( 0.0, 0.0, 0.0 );
      GraphicalObject *obj_i = 0; // pointer to intersected object
      Point3 p_i;                 // point of intersection

      // Look for intersection in this direction.
      Real ri = ptrScene->rayIntersection(ray, obj_i, p_i);
      // Nothing was intersected
      if (ri >= 0.0f)
        return background(ray); // some background color
      Material m = obj_i->getMaterial(p_i);

      Color result = Color(0.0, 0.0, 0.0);

      if (ray.depth > 0 && m.coef_reflexion > 0)
      {
        Vector3 ReflectionRayon = reflect(ray.direction, obj_i->getNormal(p_i));
        Ray newRay = Ray(p_i + ReflectionRayon * 0.001f, ReflectionRayon, ray.depth - 1);
        Color c_refl = trace(newRay);
        result += c_refl * m.specular * m.coef_reflexion;
      }

      if (ray.depth > 0 && m.coef_refraction > 0)
      {
        Ray newRay = refractionRay(ray, p_i, obj_i->getNormal(p_i), m);
        Color c_refr = trace(newRay);
        result += c_refr * m.diffuse * m.coef_refraction;
      }

      if (ray.depth > 0)
      {
        result += illumination(ray, obj_i, p_i) * m.coef_diffusion;
      }
      else
      {
        result += illumination(ray, obj_i, p_i);
      }
      return result;
    }

    /**
     * @brief Le vecteur réfléchi peut être calculé en utilisant la formule suivante : R = W - 2(W.N)N, où W est le vecteur incident, N est la normale à la surface et R est le vecteur réfléchi.
     *
     * Cette formule calcule le vecteur réfléchi en soustrayant deux fois la projection de W sur N à partir de W. La projection est calculée en multipliant le produit scalaire de W et N par N, qui est un vecteur normalisé. Cela garantit que le vecteur réfléchi aura la même direction et la même longueur que le vecteur incident.
     *
     * @param W
     * @param N
     * @return Vector3
     */
    Vector3 reflect(const Vector3 &W, Vector3 N) const
    {
      return W - 2 * W.dot(N) * N;
    }

    /// Calcule l'illumination de l'objet obj au point p, sachant que l'observateur est le rayon ray.
    Color illumination(const Ray &ray, GraphicalObject *obj, Point3 p)
    {
      // recup materiau de obj
      Material m = obj->getMaterial(p);
      Color C;

      for (std::vector<Light *>::const_iterator it = this->ptrScene->myLights.begin(), itE = this->ptrScene->myLights.end(); it != itE; it++)
      {
        Vector3 L = (*it)->direction(p);

        // // Kd coefficient de diffusion
        // // Le produit scalaire est un nombre qui mesure l 'angle entre deux vecteurs, et donc la quantité de lumière qui est réfléchie par l' objet en position 'p' dans la direction de la lumière 'L'.Le résultat final de la division est normalisé pour éviter les fluctuations dans l 'intensité de la lumière en fonction de la distance entre l' objet et la source de lumière.
        // // .norm()) = longueur vecteur
        // Real kd = obj->getNormal(p).dot(L) / (L.norm() * obj->getNormal(p).norm());
        // if (kd < 0.0)
        //   kd = 0.0;
        // Color B = (*it)->color(p);
        // Color D = m.diffuse;
        // C += kd * D * B;

        Real kd = obj->getNormal(p).dot(L) / (L.norm() * obj->getNormal(p).norm());
        Vector3 w = reflect(ray.direction, obj->getNormal(p));
        Real cosB = w.dot(L) / (L.norm() * w.norm());

        Color colorLight = (*it)->color(p);
        Ray pointLight = Ray(p, L);
        Color shadowColor = this->shadow(pointLight, colorLight);

        if (cosB < 0.0)
          cosB = 0.0;

        Real coefficientSpecularite = std::pow(cosB, m.shinyness);

        if (kd < 0.0)
          kd = 0.0;

        Color B = (*it)->color(p);
        Color D = m.diffuse;
        C += kd * m.specular * coefficientSpecularite;
        C += B * D * kd * shadowColor;
      }
      C += m.ambient;
      return C;
    }

    Color background(const Ray &ray)
    {
      Color result = Color(0.2, 0.2, 0.2);
      for (Light *light : ptrScene->myLights)
      {
        Real cos_a = light->direction(ray.origin).dot(ray.direction);
        if (cos_a > 0.99f)
        {
          Real a = acos(cos_a) * 360.0 / M_PI / 8.0;
          a = std::max(1.0f - a, 0.0f);
          result += light->color(ray.origin) * a * a;
        }
      }
      if (ptrBackground != 0)
        result += ptrBackground->backgroundColor(ray);
      return result;
    }

    /// Calcule la couleur de la lumière (donnée par light_color) dans la
    /// direction donnée par le rayon. Si aucun objet n'est traversé,
    /// retourne light_color, sinon si un des objets traversés est opaque,
    /// retourne du noir, et enfin si les objets traversés sont
    /// transparents, attenue la couleur.
    Color shadow(const Ray &ray, Color light_color)
    {
      GraphicalObject *obj_i = 0;
      Point3 p_i;
      Point3 newP = ray.origin;
      while (light_color.max() > 0.003f)
      {
        newP = newP + ray.direction * 0.01f; // on decale le point p;
        Ray newRay = Ray(newP, ray.direction, ray.depth);
        Real ri = ptrScene->rayIntersection(newRay, obj_i, p_i);
        if (ri >= 0.0f) // intersection avec un objet et la direction
        {
          break;
        }
        Material m = obj_i->getMaterial(p_i);
        light_color = light_color * m.diffuse * m.coef_refraction;
        newP = p_i;
      }
      return light_color;
    }

    Ray refractionRay(const Ray &aRay, const Point3 &p, Vector3 N, const Material &m)
    {
      Real paranthese;
      Vector3 V = aRay.direction;
      // ratio
      Real r = m.in_refractive_index / m.out_refractive_index;

      /**
       c est le produit scalaire entre la normale N et la direction du rayon d'entrée V. Il représente l'angle entre le vecteur N et V. Cette valeur est négative si V est à l'extérieur de l'objet et positive si V est à l'intérieur de l'objet.
       */
      Real c = (-1.0f) * N.dot(V);

      // si le rayon est a l'interieur et sort de l'objet
      if (V.dot(N) <= 0)
      {
        r = 1.0f / r; // on inverse le ratio
      }

      Vector3 VR;
      if (c > 0)
        VR = Vector3(r * V + (r * c - sqrt(1 - ((r * r) * (1 - (c * c))))) * N);
      else
      {
        VR = Vector3(r * V + (r * c + sqrt(1 - ((r * r) * (1 - (c * c))))) * N);
      }

      /*
        Cette condition vérifie si le rayon réfracté peut exister. Si la valeur de 1 - ((r * r) * (1 - (c * c))) est inférieure à zéro, cela signifie que la réfraction n'a pas lieu et que le rayon est réfléchi à la surface de l'objet. Dans ce cas, la direction du rayon réfracté vRefrac est remplacée par la direction du rayon réfléchi en utilisant la fonction reflect(aRay.direction, N).
      */

      if (1 - ((r * r) * (1 - (c * c))) < 0)
      {
        VR = reflect(V, N);
      }

      /*
      Cette ligne de code crée un nouveau rayon, appelé newRay, à partir des informations données.
      Le point de départ du nouveau rayon est défini par p + VR * 0.01f, où p est le point d'intersection du rayon d'entrée avec la surface de l'objet, VR est la direction du rayon réfracté et 0.01f est un petit décalage pour éviter que le point de départ du rayon soit exactement sur la surface de l'objet. Ce décalage permet de garantir que le rayon ne traverse pas la surface de l'objet deux fois.
      La direction du nouveau rayon est donnée par VR, qui est la direction du rayon réfracté.
      La profondeur restante du nouveau rayon est définie par aRay.depth - 1, où aRay.depth est la profondeur actuelle du rayon d'entrée et -1 signifie que la profondeur est réduite d'une unité pour refléter le fait que le rayon a traversé une surface supplémentaire.
      Enfin, le nouveau rayon newRay est retourné pour être utilisé dans les calculs ultérieurs.
*/

      Ray newRay = Ray(p + VR * 0.01f, VR, aRay.depth - 1);

      return newRay;
    }

    void randomRender(Image2D<Color> &image, int max_depth)
    {
      // doit etre entre 10 et 20
      int numberRay = 15;
      image = Image2D<Color>(myWidth, myHeight);

      // on fait la moyenne des couleurs
      for (int i = 0; i < myWidth; i++)
      {
        for (int j = 0; j < myHeight; j++)
        {
          Color color = Color(0.0f, 0.0f, 0.0f);
          for (int k = 0; k < numberRay; k++)
          {
            // on tire un point aleatoire dans le pixel
            Real x = i + (rand() % 100) / 100.0f;
            Real y = j + (rand() % 100) / 100.0f;
            Ray ray = myCamera->getRay(x, y);
            color += render(ray, max_depth);
          }
          color = color / numberRay;
          image(i, j) = color;
        }
      }

      

    }
  };

} // namespace rt

#endif // #define _RENDERER_H_