#include "ssspmbrdf.h"
#include <warpfunctions.h>

Color3f SSSPMBRDF::f(const Vector3f &wo, const Vector3f &wi) const
{
    //TODO
    return R*InvPi;
}

Color3f SSSPMBRDF::Sample_f(const Vector3f &wo, Vector3f *wi, const Point2f &u,
                        Float *pdf, BxDFType *sampledType) const
{
    //TODO
    //*wi = glm::normalize( WarpFunctions::squareToHemisphereUniform(u));
    *wi = glm::normalize( WarpFunctions::squareToHemisphereCosine(u));
    *pdf = Pdf(wo, *wi);
    return f(wo, *wi);
}

float SSSPMBRDF::Pdf(const Vector3f &wo, const Vector3f &wi) const
{
    //TODO
    //return SameHemisphere(wo, wi) ? WarpFunctions::squareToHemisphereUniformPDF(wi) : 0;
//    return SameHemisphere(wo, wi) ? WarpFunctions::squareToHemisphereCosinePDF(wi) : 0;
    //since this is called for the the point we see and the point on the surface with our diffusion range
    //i think it should be square to sphere cosine pdf
    return WarpFunctions::squareToSphereCosinePDF(wi);
}

Color3f SSSPMBRDF::Sample_f_photon(const Vector3f &wo, Vector3f *wi, const Point2f &u,
                        Float *pdf, BxDFType *sampledType) const
{
    //TODO
    //*wi = glm::normalize( WarpFunctions::squareToHemisphereUniform(u));
    *wi = glm::normalize( WarpFunctions::squareToHemisphereUniform(u));
    *pdf = Pdf_photon(wo, *wi);
    return f(wo, *wi);
}

float SSSPMBRDF::Pdf_photon(const Vector3f &wo, const Vector3f &wi) const
{
    //TODO
    //return SameHemisphere(wo, wi) ? WarpFunctions::squareToHemisphereUniformPDF(wi) : 0;
//    return SameHemisphere(wo, wi) ? WarpFunctions::squareToHemisphereCosinePDF(wi) : 0;
    //since this is called for the the point we see and the point on the surface with our diffusion range
    //i think it should be square to sphere cosine pdf
    return WarpFunctions::squareToSphereUniformPDF(wi);
}
