#pragma once
#include <scene/geometry/shape.h>
class ImplicitSurface: public Shape
{
public:
    virtual bool Intersect(const Ray &ray, Intersection *isect) const;
    virtual Point2f GetUVCoordinates(const Point3f &point) const;
    virtual float Area() const;
    virtual void ComputeTBN(const Point3f& P, Normal3f* nor, Vector3f* tan, Vector3f* bit) const;
    // Sample a point on the surface of the shape and return the PDF with
    // respect to area on the surface.
    virtual Intersection Sample(const Point2f &xi, Float *pdf) const;
    virtual void getSurfaceEmissionRay(const Point2f &xi, const Point2f& xk, Ray& ray ,float& pdf) const;
    Bounds3f WorldBound() const;
    void create();//defined as the sphere create() in scene/geometry/glshapecreation.cpp
};

