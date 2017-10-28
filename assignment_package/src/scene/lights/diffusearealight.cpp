#include "diffusearealight.h"

Color3f DiffuseAreaLight::L(const Intersection &isect, const Vector3f &w) const
{
    //TODO
    //LOBE PROBLEM WAS FROM CALLING SAMEHEMISPHERE INSTEAD OF DOT>0
    if (this->twoSided || glm::dot(w,isect.normalGeometric) > 0.f) {
        return emittedLight;
    } else {
        return Color3f(0.f);
    }
}

float DiffuseAreaLight::getProxyWattage() const {
    const float luminance = 0.2126f*emittedLight.r + 0.7152f*emittedLight.g + 0.0722f*emittedLight.b;
    return luminance*this->shape->Area();
}

Color3f DiffuseAreaLight::getEmittedLight() const {
   return emittedLight;
}

float DiffuseAreaLight::getLightArea() const{
    return this->shape->Area();
}

void DiffuseAreaLight::getSurfaceEmissionRay(const Point2f &xi, const Point2f& xk, Ray& ray, float& pdf) const {
    this->shape->getSurfaceEmissionRay(xi,xk,ray,pdf);
}

// Given a point of intersection on some surface in the scene and a pair
// of uniform random variables, generate a sample point on the surface
// of our shape and evaluate the light emitted along the ray from
// our Shape's surface to the reference point.
Color3f DiffuseAreaLight::Sample_Li(const Intersection &ref, const Point2f &xi,
                  Vector3f *wi, Float *pdf) const {
    Intersection lightsample = this->shape->Sample(ref,xi,pdf);
    if( (*pdf) <= 0.f || (ref.point == lightsample.point)) {
        *pdf = 0.f;
        return Color3f(0.f);
    }
    *wi = glm::normalize(lightsample.point - ref.point);
    if(*pdf > 0) {
        return this->L(lightsample, -(*wi));
    }
}

float DiffuseAreaLight::Pdf_Li(const Intersection &ref, const Vector3f &wi) const {
    //TODO
    Intersection isect_thislight = Intersection();
    Ray wiWray = ref.SpawnRay(wi);
    if(!this->shape->Intersect(wiWray,&isect_thislight)) {
        return 0.f;
    }

    float coslight = glm::dot(isect_thislight.normalGeometric, -wi);
    if(this->twoSided){
        coslight = glm::abs(coslight);
    }
    float denominator = coslight * this->shape->Area();
    if(denominator > 0.f) {
        return glm::distance2(ref.point, isect_thislight.point) / denominator;
    } else {
        return 0.f;
    }
}

