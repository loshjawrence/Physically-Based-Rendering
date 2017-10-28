#include "disc.h"
#include <warpfunctions.h>

float Disc::Area() const
{
    //TODO
    return Pi * transform.getScale().x * transform.getScale().x; //assume uniform scaling
}

Intersection Disc::Sample(const Point2f &xi, Float *pdf) const
{
    Intersection isect;
    Point3f p = WarpFunctions::squareToDiskConcentric(xi);
    glm::vec4 pworld = transform.T() * glm::vec4(p.x, p.y, p.z,1.0f);
    Point3f pworld3 = Point3f(pworld.x, pworld.y, pworld.z);
    isect.point = pworld3;
    isect.normalGeometric = glm::normalize(transform.invTransT() * Normal3f(0,0,1));

    *pdf = 1 / Area();
    return isect;
}

Bounds3f Disc::WorldBound() const {
    Bounds3f modelbounds(Point3f(-1.f, -1.f, -0.005f), Point3f(1.f, 1.f, 0.005f));
    return modelbounds.Apply(this->transform);
}

void Disc::getSurfaceEmissionRay(const Point2f &xi, const Point2f& xk, Ray& ray ,float& pdf) const {
    pdf = 1.f / Area();
    const Point3f surfpoint = WarpFunctions::squareToDiskConcentric(xi);
    const glm::vec4 p                 = transform.T() * glm::vec4(surfpoint.x, surfpoint.y, 0.005f, 1.f);
    ray.origin = Vector3f(p.x, p.y, p.z);
    const Vector3f dir = WarpFunctions::squareToHemisphereUniform(xk);
    ray.direction      = glm::normalize(transform.invTransT() * dir);
}

bool Disc::Intersect(const Ray &ray, Intersection *isect) const
{
    //Transform the ray
    Ray r_loc = ray.GetTransformedCopy(transform.invT());

    //Ray-plane intersection
    float t = glm::dot(glm::vec3(0,0,1), (glm::vec3(0.5f, 0.5f, 0) - r_loc.origin)) / glm::dot(glm::vec3(0,0,1), r_loc.direction);
    Point3f P = Point3f(t * r_loc.direction + r_loc.origin);
    //Check that P is within the bounds of the disc (not bothering to take the sqrt of the dist b/c we know the radius)
    float dist2 = (P.x * P.x + P.y * P.y);
    if(t > 0 && dist2 <= 1.f)
    {
        InitializeIntersection(isect, t, P);
        return true;
    }
    return false;
}

void Disc::ComputeTBN(const Point3f &P, Normal3f *nor, Vector3f *tan, Vector3f *bit) const
{
    *nor = glm::normalize(transform.invTransT() * Normal3f(0,0,1));
    *tan = glm::normalize(transform.T3() * Vector3f(1,0,0));
    *bit = glm::normalize(transform.T3() * Vector3f(0,1,0));
}


Point2f Disc::GetUVCoordinates(const Point3f &point) const
{
    return glm::vec2((point.x + 1)/2.f, (point.y + 1)/2.f);
}
