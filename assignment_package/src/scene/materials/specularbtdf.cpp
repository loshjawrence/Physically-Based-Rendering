#include "specularbTdf.h"

Color3f SpecularBTDF::f(const Vector3f &wo, const Vector3f &wi) const
{
    return Color3f(0.f);
}


float SpecularBTDF::Pdf(const Vector3f &wo, const Vector3f &wi) const
{
    return 0.f;
}

Color3f SpecularBTDF::Sample_f(const Vector3f &wo, Vector3f *wi, const Point2f &sample, Float *pdf, BxDFType *sampledType) const
{
    //genType glm::refract	(	genType const & 	I,
    //genType const & 	N,
    //typename genType::value_type const & 	eta
    //)
    //For the incident vector I and surface normal N, and the ratio of indices of refraction eta, return the refraction vector.
//    float eta_ratio = 1.f;
//    *pdf = 1.f;
//    Normal3f nor(0.f, 0.f, 1.f);
//    if( glm::dot(wo, nor) >= 0.f) {//entering
//        eta_ratio = etaA / etaB;
//        *wi = glm::normalize( glm::refract(wo, nor, eta_ratio) );
//    } else {
//        eta_ratio = etaB / etaA;
//        *wi = glm::normalize( glm::refract(wo, -nor, eta_ratio) );

//        //internal reflection
//        if(SameHemisphere(*wi, -nor)) {
//            return Color3f(0.f);
//        }
//    }

    //page 529-531 in pbrt
    //figure out which n is incident and which is transmitted
    bool entering = CosTheta(wo) > 0.f;
    float etaI = entering ? etaA : etaB;
    float etaT = entering ? etaB : etaA;

    //compute ray direction for specular transmission, return 0 if total internal reflection
    if(!Refract(wo, Faceforward(Normal3f(0,0,1), wo), etaI / etaT, wi)) {
        return Color3f(0.f);
    }
    *pdf = 1;
    Color3f t = this->T * (1.f - this->fresnel->Evaluate(CosTheta(*wi)));
    return t / AbsCosTheta(*wi);

}


float SpecularBTDF::Pdf_photon(const Vector3f &wo, const Vector3f &wi) const
{
    return 0.f;
}

Color3f SpecularBTDF::Sample_f_photon(const Vector3f &wo, Vector3f *wi, const Point2f &sample, Float *pdf, BxDFType *sampledType) const
{
    //genType glm::refract	(	genType const & 	I,
    //genType const & 	N,
    //typename genType::value_type const & 	eta
    //)
    //For the incident vector I and surface normal N, and the ratio of indices of refraction eta, return the refraction vector.
//    float eta_ratio = 1.f;
//    *pdf = 1.f;
//    Normal3f nor(0.f, 0.f, 1.f);
//    if( glm::dot(wo, nor) >= 0.f) {//entering
//        eta_ratio = etaA / etaB;
//        *wi = glm::normalize( glm::refract(wo, nor, eta_ratio) );
//    } else {
//        eta_ratio = etaB / etaA;
//        *wi = glm::normalize( glm::refract(wo, -nor, eta_ratio) );

//        //internal reflection
//        if(SameHemisphere(*wi, -nor)) {
//            return Color3f(0.f);
//        }
//    }

    //page 529-531 in pbrt
    //figure out which n is incident and which is transmitted
    bool entering = CosTheta(wo) > 0.f;
    float etaI = entering ? etaA : etaB;
    float etaT = entering ? etaB : etaA;

    //compute ray direction for specular transmission, return 0 if total internal reflection
    if(!Refract(wo, Faceforward(Normal3f(0,0,1), wo), etaI / etaT, wi)) {
        return Color3f(0.f);
    }
    *pdf = 1;
    Color3f t = this->T * (1.f - this->fresnel->Evaluate(CosTheta(*wi)));
    return t / AbsCosTheta(*wi);

}
