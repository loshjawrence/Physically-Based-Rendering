#pragma once
#include "integrator.h"
#include <scene/photonmap.h>

class PhotonMapIntegrator : public Integrator
{
public:
    PhotonMapIntegrator(Bounds2i bounds, Scene* s, std::shared_ptr<Sampler> sampler, int recursionLimit)
        : Integrator(bounds, s, sampler, recursionLimit)
    {}

    // Evaluate the energy transmitted along the ray back to
    // its origin using multiple importance sampling
    virtual Color3f Li(const Ray &ray, const Scene &scene, std::shared_ptr<Sampler> sampler, int depth) const;
    Color3f integratePhotonMap(const PHOTONTYPE type, const Vector3f& woW, const Intersection& isect, const bool isSSSPM) const;

    Color3f calcIndirectLightingFromPhotonMaps(Color3f& accumcolor,
       const Vector3f& woW, const Intersection& isect, const Scene& scene, const Color3f& throughput, const int bounces, const int depth, std::shared_ptr<Sampler> sampler) const;

    Color3f calcSSSPM(Color3f& accumcolor, const Ray& theray,
       const Vector3f& woW, const Intersection& isect, const Scene& scene, const Color3f& throughput, const int bounces, const int depth, std::shared_ptr<Sampler> sampler) const;

    void traceUntilNotSpecular(Intersection& isect, Color3f& accumcolor, Color3f& throughput, Ray& woWray, int& spechits, bool& continueflag) const;

    float BalanceHeuristic(int nf, Float fPdf, int ng, Float gPdf) const;

    float PowerHeuristic(int nf, Float fPdf, int ng, Float gPdf) const;
};

