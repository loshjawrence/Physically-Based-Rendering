#pragma once
#include <QList>
#include <raytracing/film.h>
#include <scene/camera.h>
#include <scene/camerathinfilm.h>
#include <scene/lights/light.h>
#include <scene/geometry/shape.h>
#include "bvh.h"
#include <scene/photonmap.h>

class Primitive;
class BVHAccel;
class Material;
class Light;
class PhotonMap;

class Scene
{
public:
    Scene();
    ~Scene();
    QList<std::shared_ptr<Primitive>> primitives;
    QList<std::shared_ptr<Material>> materials;
    QList<std::shared_ptr<Light>> lights;
    Camera camera;
//    CameraThinFilm cameratf;
    Film film;

    BVHAccel* bvh;
    PhotonMap* pm;

    QList<std::shared_ptr<Drawable>> drawables;

    void SetCamera(const Camera &c);
//    void SetCamera(const CameraThinFilm &c);

    void CreateTestScene();
    void Clear();

    bool Intersect(const Ray& ray, Intersection* isect) const;

    void clearBVH();
    void clearPM();

};
