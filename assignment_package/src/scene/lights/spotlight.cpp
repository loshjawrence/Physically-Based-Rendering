#include "spotlight.h"

Color3f SpotLight::L(const Intersection &isect, const Vector3f &w) const
{
    return Color3f(0,0,0);
}

float SpotLight::getProxyWattage() const {
    const float luminance = 0.2126f*emittedLight.r + 0.7152f*emittedLight.g + 0.0722f*emittedLight.b;
    return luminance;
}

Color3f SpotLight::getEmittedLight() const {
   return emittedLight;
}
float SpotLight::getLightArea() const{
    return 1.f;
}
void SpotLight::getSurfaceEmissionRay(const Point2f &xi, const Point2f& xk, Ray& ray, float& pdf) const {
    //same as pointlight for now
    ray.origin = this->pLight;
    ray.direction = WarpFunctions::squareToSphereUniform(xi);
    pdf = WarpFunctions::squareToSphereUniformPDF(ray.direction);
}



// Given a point of intersection on some surface in the scene and a pair
// of uniform random variables, generate a sample point on the surface
// of our shape and evaluate the light emitted along the ray from
// our Shape's surface to the reference point.
Color3f SpotLight::Sample_Li(const Intersection &ref, const Point2f &xi,
                  Vector3f *wi, Float *pdf) const {
    *wi = glm::normalize(pLight - ref.point);
    *pdf = 1.f;
    return this->emittedLight * Falloff(-(*wi)) * (1.f / glm::distance2(pLight,ref.point));
}

Float SpotLight::Falloff(const Vector3f& w) const {
    Vector3f wl = glm::normalize(glm::vec3(transform.invT()*glm::vec4(w,0)));
    Float cosTheta = wl.z;
    if(cosTheta < cosTotalWidth) return 0;
    if(cosTheta > cosFalloffStart) return 1;
    Float delta = (cosTheta - cosTotalWidth) / (cosFalloffStart - cosTotalWidth);
    return (delta * delta) * (delta * delta);
}

float SpotLight::Pdf_Li(const Intersection &ref, const Vector3f &wi) const {
    return 0.f;
}

