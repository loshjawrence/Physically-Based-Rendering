#include "fresnel.h"

Color3f FresnelDielectric::Evaluate(float cosThetaI) const
{
    //TODO
    float etaI_temp = etaI; float etaT_temp = etaT;
    cosThetaI = glm::clamp(cosThetaI, -1.f, 1.f);
    //potentially swap indices of refraction
    bool entering = cosThetaI > 0.f;
    if(!entering) {
        float temp = etaI_temp;
        etaI_temp = etaT_temp;
        etaT_temp = temp;
        cosThetaI = std::abs(cosThetaI);
    }

    //compute cos theta I using snells law
    float sinThetaI = std::sqrt(std::max(0.f, 1.f - cosThetaI * cosThetaI));
    float sinThetaT = etaI_temp / etaT_temp * sinThetaI;
    //handle internal reflection
    if(sinThetaT >= 1) {
        return Color3f(1.f);
    }
    float cosThetaT = std::sqrt(std::max(0.f, 1.f - sinThetaT * sinThetaT));

    //equations from 518 in pbrt for calculating fresnel reflectance
    //at the interface of two dialectrics
    float r_parallel      = ((etaT_temp * cosThetaI) - (etaI_temp * cosThetaT)) /
                            ((etaT_temp * cosThetaI) + (etaI_temp * cosThetaT));
    float r_perpendicular = ((etaI_temp * cosThetaI) - (etaT_temp * cosThetaT)) /
                            ((etaI_temp * cosThetaI) + (etaT_temp * cosThetaT));
    float Fr = (r_parallel * r_parallel + r_perpendicular * r_perpendicular) / 2.f;
    return Color3f(Fr);
}
