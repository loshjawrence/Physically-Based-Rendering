#pragma once
#include "bsdf.h"

class SSSPMBRDF : public BxDF
{
public:
    SSSPMBRDF(const Color3f &R)
        : BxDF(BxDFType(BSDF_REFLECTION | BSDF_DIFFUSE | BSDF_SSSPM)), R(R) {}

    Color3f f(const Vector3f &wo, const Vector3f &wi) const;

    virtual Color3f Sample_f(const Vector3f &wo, Vector3f *wi,
                              const Point2f &sample, Float *pdf,
                              BxDFType *sampledType = nullptr) const;
    virtual float Pdf(const Vector3f &wo, const Vector3f &wi) const;

    virtual Color3f Sample_f_photon(const Vector3f &wo, Vector3f *wi,
                              const Point2f &sample, Float *pdf,
                              BxDFType *sampledType = nullptr) const;
    virtual float Pdf_photon(const Vector3f &wo, const Vector3f &wi) const;


  private:
    const Color3f R; // The energy scattering coefficient of this BRDF (i.e. its color)
};
