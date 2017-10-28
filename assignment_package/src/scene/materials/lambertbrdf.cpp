#include "lambertbrdf.h"
#include <warpfunctions.h>

Color3f LambertBRDF::f(const Vector3f &wo, const Vector3f &wi) const
{
    //TODO
    return R*InvPi;
}

Color3f LambertBRDF::Sample_f(const Vector3f &wo, Vector3f *wi, const Point2f &u,
                        Float *pdf, BxDFType *sampledType) const
{
    //TODO
    //*wi = glm::normalize( WarpFunctions::squareToHemisphereUniform(u));
    *wi = glm::normalize( WarpFunctions::squareToHemisphereCosine(u));
    *pdf = Pdf(wo, *wi);
    return f(wo, *wi);
}

float LambertBRDF::Pdf(const Vector3f &wo, const Vector3f &wi) const
{
    //TODO
    //return SameHemisphere(wo, wi) ? WarpFunctions::squareToHemisphereUniformPDF(wi) : 0;
    return SameHemisphere(wo, wi) ? WarpFunctions::squareToHemisphereCosinePDF(wi) : 0;
}


Color3f LambertBRDF::Sample_f_photon(const Vector3f &wo, Vector3f *wi, const Point2f &u,
                        Float *pdf, BxDFType *sampledType) const
{
    //TODO
    //*wi = glm::normalize( WarpFunctions::squareToHemisphereUniform(u));
    *wi = glm::normalize( WarpFunctions::squareToHemisphereUniform(u));
    *pdf = Pdf_photon(wo, *wi);
    return f(wo, *wi);
}

float LambertBRDF::Pdf_photon(const Vector3f &wo, const Vector3f &wi) const
{
    //TODO
    //return SameHemisphere(wo, wi) ? WarpFunctions::squareToHemisphereUniformPDF(wi) : 0;
    return SameHemisphere(wo, wi) ? WarpFunctions::squareToHemisphereUniformPDF(wi) : 0;
}
