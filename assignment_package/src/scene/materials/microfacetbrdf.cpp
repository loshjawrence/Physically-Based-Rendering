#include "microfacetbrdf.h"
#include <warpfunctions.h>

Color3f MicrofacetBRDF::f(const Vector3f &wo, const Vector3f &wi) const
{
    float cosThetaO = AbsCosTheta(wo);
    float cosThetaI = AbsCosTheta(wi);

    Vector3f wh = wi + wo;

    if(cosThetaI == 0.0f || cosThetaO == 0.0f) {
        return Color3f(0.f);
    }

    if(wh.x == 0.0f || wh.y == 0.0f || wh.z == 0.0f) {
        return Color3f(0.f);
    }

    wh = glm::normalize(wh);

    Color3f F = fresnel->Evaluate(glm::dot(wi, wh));
    return R * distribution->D(wh) * distribution->G(wo, wi) * F / (4.0f * cosThetaI * cosThetaO);
}

Color3f MicrofacetBRDF::Sample_f(const Vector3f &wo, Vector3f *wi, const Point2f &xi, Float *pdf, BxDFType *sampledType) const
{
    //TODO
    Vector3f wh = distribution->Sample_wh(wo, xi);

//   reflection = 2*dot(wo, wh)*wh - wo;
    *wi = glm::reflect(-wo,wh);

    if(!SameHemisphere(wo, *wi)) {
        return Color3f(0.0f);
    }

    *pdf = Pdf(wo, *wi);
    return f(wo, *wi);
}

float MicrofacetBRDF::Pdf(const Vector3f &wo, const Vector3f &wi) const
{
    //TODO        
    if(!SameHemisphere(wo, wi)) {
        return 0.0f;
    }

     Vector3f wh = glm::normalize(wi + wo);
    return distribution->Pdf(wo, wh) / (4.0f * glm::dot(wo, wh));
}

Color3f MicrofacetBRDF::Sample_f_photon(const Vector3f &wo, Vector3f *wi, const Point2f &xi, Float *pdf, BxDFType *sampledType) const
{
    //TODO
    Vector3f wh = distribution->Sample_wh(wo, xi);

//   reflection = 2*dot(wo, wh)*wh - wo;
    *wi = glm::reflect(-wo,wh);

    if(!SameHemisphere(wo, *wi)) {
        return Color3f(0.0f);
    }

    *pdf = Pdf_photon(wo, *wi);
    return f(wo, *wi);
}

float MicrofacetBRDF::Pdf_photon(const Vector3f &wo, const Vector3f &wi) const
{
    //TODO
    if(!SameHemisphere(wo, wi)) {
        return 0.0f;
    }

     Vector3f wh = glm::normalize(wi + wo);
    return distribution->Pdf(wo, wh) / (4.0f * glm::dot(wo, wh));
}
