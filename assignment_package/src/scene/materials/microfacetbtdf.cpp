#include "microfacetbtdf.h"

Color3f MicrofacetBTDF::f(const Vector3f &wo, const Vector3f &wi) const
{
        //taken from pbrt's github
    if (SameHemisphere(wo, wi))
        return Color3f(0.0f);  // transmission only

        float cosThetaO = CosTheta(wo);
        float cosThetaI = CosTheta(wi);
        if (cosThetaI == 0 || cosThetaO == 0) {
            return Color3f(0.0f);
        }


        float eta = CosTheta(wo) > 0 ? (etaB / etaA) : (etaA / etaB);
        Vector3f wh = glm::normalize(wo + wi * eta);

        if (wh.z < 0) {
            wh = -wh;
        }

        Color3f F = fresnel->Evaluate(glm::dot(wo,wh));
        float sqrtDenom = glm::dot(wo,wh) + eta * glm::dot(wi,wh);


        return (Color3f(1.0f) - F) * T *
                glm::abs(distribution->D(wh) * distribution->G(wo, wi) * eta * eta *
                         AbsDot(wi,wh) * AbsDot(wo,wh) /  (cosThetaI * cosThetaO * sqrtDenom * sqrtDenom));
}

Color3f MicrofacetBTDF::Sample_f(const Vector3f &wo, Vector3f *wi, const Point2f &xi, Float *pdf, BxDFType *sampledType) const
{
   if (wo.z == 0.f) return Color3f(0.f);//from pbrt's github
    Vector3f wh = distribution->Sample_wh(wo, xi);
    float eta = CosTheta(wo) > 0.0f ? (etaA / etaB) : (etaB / etaA);

    if(!Refract(wo, wh, eta, wi)) {
        return Color3f(0.0f);
    }

    *pdf = Pdf(wo, *wi);
    return f(wo, *wi);
}

float MicrofacetBTDF::Pdf(const Vector3f &wo, const Vector3f &wi) const
{
    //TODO
    if(SameHemisphere(wo, wi)) {
        return 0.f;
    }

    float eta = CosTheta(wo) > 0 ? (etaB / etaA) : (etaA / etaB);
    Vector3f wh = glm::normalize(wo + wi*eta);
    // dwh_dwi found off of pbrt's github
    //Compute change of variables _dwh\_dwi_ for microfacet transmission
    Float sqrtDenom = glm::dot(wo, wh) + eta * glm::dot(wi, wh);
    Float dwh_dwi =
        std::abs((eta * eta * glm::dot(wi, wh)) / (sqrtDenom * sqrtDenom));

    return distribution->Pdf(wo, wh) * dwh_dwi;
}

Color3f MicrofacetBTDF::Sample_f_photon(const Vector3f &wo, Vector3f *wi, const Point2f &xi, Float *pdf, BxDFType *sampledType) const
{
   if (wo.z == 0.f) return Color3f(0.f);//from pbrt's github
    Vector3f wh = distribution->Sample_wh(wo, xi);
    float eta = CosTheta(wo) > 0.0f ? (etaA / etaB) : (etaB / etaA);

    if(!Refract(wo, wh, eta, wi)) {
        return Color3f(0.0f);
    }

    *pdf = Pdf_photon(wo, *wi);
    return f(wo, *wi);
}

float MicrofacetBTDF::Pdf_photon(const Vector3f &wo, const Vector3f &wi) const
{
    //TODO
    if(SameHemisphere(wo, wi)) {
        return 0.f;
    }

    float eta = CosTheta(wo) > 0 ? (etaB / etaA) : (etaA / etaB);
    Vector3f wh = glm::normalize(wo + wi*eta);
    // dwh_dwi found off of pbrt's github
    //Compute change of variables _dwh\_dwi_ for microfacet transmission
    Float sqrtDenom = glm::dot(wo, wh) + eta * glm::dot(wi, wh);
    Float dwh_dwi =
        std::abs((eta * eta * glm::dot(wi, wh)) / (sqrtDenom * sqrtDenom));

    return distribution->Pdf(wo, wh) * dwh_dwi;
}
