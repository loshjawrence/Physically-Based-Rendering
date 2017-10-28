#include "specularbrdf.h"

Color3f SpecularBRDF::f(const Vector3f &wo, const Vector3f &wi) const
{
    return Color3f(0.f);
}


float SpecularBRDF::Pdf(const Vector3f &wo, const Vector3f &wi) const
{
    return 0.f;
}

Color3f SpecularBRDF::Sample_f(const Vector3f &wo, Vector3f *wi, const Point2f &sample, Float *pdf, BxDFType *sampledType) const
{
    *wi = Vector3f(-wo.x, -wo.y, wo.z);
    *pdf = 1.f;
//    return this->R;
    //525 in pbrt
    return this->fresnel->Evaluate(CosTheta(*wi)) * R / AbsCosTheta(*wi);


}


float SpecularBRDF::Pdf_photon(const Vector3f &wo, const Vector3f &wi) const
{
    return 0.f;
}

Color3f SpecularBRDF::Sample_f_photon(const Vector3f &wo, Vector3f *wi, const Point2f &sample, Float *pdf, BxDFType *sampledType) const
{
    *wi = Vector3f(-wo.x, -wo.y, wo.z);
    *pdf = 1.f;
//    return this->R;
    //525 in pbrt
    return this->fresnel->Evaluate(CosTheta(*wi)) * R / AbsCosTheta(*wi);


}
