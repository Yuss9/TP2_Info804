#include <qapplication.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include "Viewer.h"
#include "Scene.h"
#include "Sphere.h"
#include "Material.h"
#include "PointLight.h"
#include "PeriodicPlane.hpp"

using namespace std;
using namespace rt;

void addEmeraldInGlassBubble(Scene &scene, Point3 c, Real r_outer, Real r_inner)
{
  Sphere *sphere_outer = new Sphere(c, 2.0, Material::glass());
  Sphere *sphere_inner = new Sphere(c, 1.0, Material::emerald());
  scene.addObject(sphere_outer);
  scene.addObject(sphere_inner);
}

void addBubble(Scene &scene, Point3 c, Real r, Material transp_m)
{
  Material revert_m = transp_m;
  std::swap(revert_m.in_refractive_index, revert_m.out_refractive_index);
  Sphere *sphere_out = new Sphere(c, r, transp_m);
  Sphere *sphere_in = new Sphere(c, r - 0.02f, revert_m);
  scene.addObject(sphere_out);
  scene.addObject(sphere_in);
}

void oldScene()
{
  // Read command lines arguments.
  //QApplication application(argc, argv);

  // Creates a 3D scene
  Scene scene;
  Light *light0 = new PointLight(GL_LIGHT0, Point4(0, 0, 1, 0),
                                 Color(1.0, 1.0, 1.0));
  Light *light1 = new PointLight(GL_LIGHT1, Point4(-10, -4, 2, 1),
                                 Color(1.0, 1.0, 1.0));
  scene.addLight(light0);
  scene.addLight(light1);
  // Objects
  Sphere *bronze1 = new Sphere(Point3(0, 0, 0), 2.0, Material::bronze());
  Sphere *bronze2 = new Sphere(Point3(1, 0, 0), 2.0, Material::bronze());
  Sphere *bronze3 = new Sphere(Point3(0, 1, 1), 2.0, Material::bronze());

  Sphere *sphere2 = new Sphere(Point3(0, 4, 0), 1.0, Material::emerald());
  Sphere *sphere3 = new Sphere(Point3(6, 6, 0), 3.0, Material::whitePlastic());

  scene.addObject(bronze1);
  scene.addObject(bronze2);
  scene.addObject(bronze3);

  scene.addObject(sphere2);
  scene.addObject(sphere3);

  addBubble(scene, Point3(-5, 10, -1), 2.0, Material::glass());
  addBubble(scene, Point3(-3, 6, -1), 2.0, Material::glass());
  addBubble(scene, Point3(-1, 8, -5), 2.0, Material::glass());
  addBubble(scene, Point3(-5, 9, -3), 2.0, Material::glass());

  addEmeraldInGlassBubble(scene, Point3(20, 20, 20), 2.0, 1.5);
  addEmeraldInGlassBubble(scene, Point3(10, 2, 20), 2.0, 1.5);
  addEmeraldInGlassBubble(scene, Point3(20, 10, 20), 2.0, 1.5);
  addEmeraldInGlassBubble(scene, Point3(15, 20, 10), 2.0, 1.5);
  addEmeraldInGlassBubble(scene, Point3(20, 20, 6), 2.0, 1.5);
  addEmeraldInGlassBubble(scene, Point3(20, 15, 20), 2.0, 1.5);
  addEmeraldInGlassBubble(scene, Point3(20, 5, 20), 2.0, 1.5);

  // Instantiate the viewer.
  Viewer viewer;
  // Give a name
  viewer.setWindowTitle("Ray-tracer preview");

  // Sets the scene
  viewer.setScene(scene);

  // Make the viewer window visible on screen.
  viewer.show();
  // Run main loop.
  //application.exec();
}

int main(int argc, char **argv)
{
  // Read command lines arguments.
  QApplication application(argc, argv);

  // Creates a 3D scene
  Scene scene;
  Light *light0 = new PointLight(GL_LIGHT0, Point4(0, 0, 1, 0),
                                 Color(1.0, 1.0, 1.0));
  Light *light1 = new PointLight(GL_LIGHT1, Point4(-10, -4, 2, 1),
                                 Color(1.0, 1.0, 1.0));
  scene.addLight(light0);
  scene.addLight(light1);

  Sphere *sphere1 = new Sphere(Point3(0, 0, 5), 2.0, Material::bronze());
  Sphere *sphere2 = new Sphere(Point3(0, 4, 5), 1.0, Material::emerald());
  Sphere *sphere3 = new Sphere(Point3(6, 6, 5), 3.0, Material::whitePlastic());
  scene.addObject(sphere1);
  scene.addObject(sphere2);
  scene.addObject(sphere3);
  addBubble(scene, Point3(-5, 4, 6), 2.0, Material::glass());

  // Un sol noir et blanc
  PeriodicPlane *pplane = new PeriodicPlane(Point3(0, 0, 0), Vector3(5, 0, 0), Vector3(0, 5, 0),
                                            Material::whitePlastic(), Material::redPlastic(), 0.05f);

  scene.addObject(pplane);
  // Instantiate the viewer.
  Viewer viewer;
  // Give a name
  viewer.setWindowTitle("Ray-tracer preview");

  // Sets the scene
  viewer.setScene(scene);

  // Make the viewer window visible on screen.
  viewer.show();
  // Run main loop.
  application.exec();
  return 0;
}