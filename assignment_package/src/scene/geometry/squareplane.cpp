#include "squareplane.h"

float SquarePlane::Area() const
{
    //TODO
    return transform.getScale().x * transform.getScale().y;
}

Intersection SquarePlane::Sample(const Point2f &xi, Float *pdf) const {
    *pdf = 1.f / Area();
    Intersection sample = Intersection();
    sample.normalGeometric      = glm::normalize(transform.invTransT() * Normal3f(0,0,1));
    glm::vec4 p                 = transform.T() * glm::vec4(xi.x - 0.5f, xi.y - 0.5f, 0.f, 1.f);
    sample.point = Vector3f(p.x, p.y, p.z);
    return sample;
}

Bounds3f SquarePlane::WorldBound() const {
    Bounds3f modelbounds(Point3f(-0.5f, -0.5f, -0.005f), Point3f(0.5f, 0.5f, 0.005f));
    return modelbounds.Apply(this->transform);
}


void SquarePlane::getSurfaceEmissionRay(const Point2f& xi, const Point2f& xk, Ray& ray ,float& pdf) const {
    pdf = 1.f / Area();
    const glm::vec4 p                 = transform.T() * glm::vec4(xi.x - 0.5f, xi.y - 0.5f, 0.005f, 1.f);
    ray.origin = Vector3f(p.x, p.y, p.z);
//    const Vector3f dir = WarpFunctions::squareToHemisphereCosine(xk);
    const Vector3f dir = WarpFunctions::squareToHemisphereUniform(xk);
    ray.direction      = glm::normalize(transform.invTransT() * dir);
}

bool SquarePlane::Intersect(const Ray &ray, Intersection *isect) const
{
    //Transform the ray
    Ray r_loc = ray.GetTransformedCopy(transform.invT());

    //Ray-plane intersection
    float t = glm::dot(glm::vec3(0,0,1), (glm::vec3(0.5f, 0.5f, 0) - r_loc.origin)) / glm::dot(glm::vec3(0,0,1), r_loc.direction);
    Point3f P = Point3f(t * r_loc.direction + r_loc.origin);
    //Check that P is within the bounds of the square
    if(t > 0 && P.x >= -0.5f && P.x <= 0.5f && P.y >= -0.5f && P.y <= 0.5f)
    {
        InitializeIntersection(isect, t, P);
        return true;
    }
    return false;
}

void SquarePlane::ComputeTBN(const Point3f &P, Normal3f *nor, Vector3f *tan, Vector3f *bit) const
{
    *nor = glm::normalize(transform.invTransT() * Normal3f(0,0,1));
    *tan = glm::normalize(transform.T3() * Vector3f(1,0,0));
    *bit = glm::normalize(transform.T3() * Vector3f(0,1,0));
}


Point2f SquarePlane::GetUVCoordinates(const Point3f &point) const
{
    return Point2f(point.x + 0.5f, point.y + 0.5f);
}
