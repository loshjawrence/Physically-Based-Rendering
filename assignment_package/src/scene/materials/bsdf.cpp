#include "bsdf.h"
#include <warpfunctions.h>

BSDF::BSDF(const Intersection& isect, float eta /*= 1*/)
    : worldToTangent(),
      tangentToWorld(),
      normal(isect.normalGeometric),
      eta(eta),
      numBxDFs(0),
      bxdfs{nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr}
{
    this->UpdateTangentSpaceMatrices(isect.normalGeometric, isect.tangent, isect.bitangent);
}

BSDF::~BSDF()
{
    for(int i = 0; i < numBxDFs; i++)
        delete bxdfs[i];
}

void BSDF::UpdateTangentSpaceMatrices(const Normal3f& n, const Vector3f& t, const Vector3f b)
{
    this->tangentToWorld[0] = t;
    this->tangentToWorld[1] = b;
    this->tangentToWorld[2] = n;

    this->worldToTangent = glm::inverse(this->tangentToWorld);
}


// Use the input random number _xi_ to select
// one of our BxDFs that matches the _type_ flags.

// After selecting our random BxDF, rewrite the first uniform
// random number contained within _xi_ to another number within
// [0, 1) so that we don't bias the _wi_ sample generated from
// BxDF::Sample_f.

// Convert woW and wiW into tangent space and pass them to
// the chosen BxDF's Sample_f (along with pdf).
// Store the color returned by BxDF::Sample_f and convert
// the _wi_ obtained from this function back into world space.

// Iterate over all BxDFs that we DID NOT select above (so, all
// but the one sampled BxDF) and add their PDFs to the PDF we obtained
// from BxDF::Sample_f, then average them all together.

// Finally, iterate over all BxDFs and sum together the results of their
// f() for the chosen wo and wi, then return that sum.

Color3f BSDF::Sample_f(const Vector3f &woW, Vector3f *wiW, const Point2f &xi, float r,
                       float *pdf, BxDFType type, BxDFType *sampledType) const
{
    //TODO
    std::vector<BxDF*> filtered;
    for(uint32_t i = 0; i < numBxDFs; ++i) {
        if(bxdfs[i]->MatchesFlags(type)) {
            filtered.push_back(bxdfs[i]);
        }
    }

    float temp = r * filtered.size();
    uint32_t index = temp;
    const BxDF* const bxdf_for_sampling = filtered[index];

    Color3f color(0.f);
    //check if not set already (directlighting will set wiW towards a random light)
    //probably old code and not needed
    if( wiW->x == 0.f && wiW->y == 0.f && wiW->z == 0.f) {
        //Don't want to bias sampling direction based on our chosen bxdf

        //sample_f and f of bxdf's done in tangent space
        Vector3f woT = worldToTangent * woW;
        Vector3f wiT(0.f);

        //this just for getting a wi
        color = bxdf_for_sampling->Sample_f(woT, &wiT, xi, pdf, sampledType);
        *wiW = tangentToWorld * wiT;
    }
    float avepdf = 1.f;
    if( (bxdf_for_sampling->type & BSDF_SPECULAR) != BSDF_SPECULAR ) {
        //call BSDF::f to get sum of all BxDFs' f() function
        //try passing sampledtype here
        color = BSDF::f(woW, *wiW, type);

        //call BSDF::Pdf to get average pdf of all BxDFs
        avepdf = BSDF::Pdf(woW, *wiW, type);
    } else {
        avepdf /= filtered.size();
    }

    *pdf = avepdf;
    return color ;

}

Color3f BSDF::f(const Vector3f &woW, const Vector3f &wiW, BxDFType flags /*= BSDF_ALL*/) const
{
    Color3f out(0.f);

    Vector3f tangentWi = worldToTangent * wiW;
    Vector3f tangentWo = worldToTangent * woW;

    for(int i = 0; i < this->numBxDFs; i++)
        if(this->bxdfs[i]->MatchesFlags(flags))
            out += this->bxdfs[i]->f(tangentWo, tangentWi);

    //should this be the average?
    return out;
}

float BSDF::Pdf(const Vector3f &woW, const Vector3f &wiW, BxDFType flags) const
{
    float pdf = 0;
    int count = 0;

    Vector3f tangentWi = worldToTangent * wiW;
    Vector3f tangentWo = worldToTangent * woW;

    for(int i = 0; i < this->numBxDFs; i++)
    {
        if(this->bxdfs[i]->MatchesFlags(flags))
        {
            pdf += this->bxdfs[i]->Pdf(tangentWo, tangentWi);
            count++;
        }
    }

    return pdf / count;
}

Color3f BxDF::Sample_f(const Vector3f &wo, Vector3f *wi, const Point2f &xi,
                       Float *pdf, BxDFType *sampledType) const
{
    *wi = WarpFunctions::squareToHemisphereUniform(xi);
    *pdf = Pdf(wo, *wi);
    return this->f(wo, *wi);
}

// The PDF for uniform hemisphere sampling
float BxDF::Pdf(const Vector3f &wo, const Vector3f &wi) const
{
    return SameHemisphere(wo, wi) ? Inv2Pi : 0;
}


Color3f BSDF::Sample_f_photon(const Vector3f &woW, Vector3f *wiW, const Point2f &xi, float r,
                       float *pdf, BxDFType type, BxDFType *sampledType) const
{
    //TODO
    std::vector<BxDF*> filtered;
    for(uint32_t i = 0; i < numBxDFs; ++i) {
        if(bxdfs[i]->MatchesFlags(type)) {
            filtered.push_back(bxdfs[i]);
        }
    }

    float temp = r * filtered.size();
    uint32_t index = temp;
    const BxDF* const bxdf_for_sampling = filtered[index];

    Color3f color(0.f);
    //check if not set already (directlighting will set wiW towards a random light)
    //probably old code and not needed
    if( wiW->x == 0.f && wiW->y == 0.f && wiW->z == 0.f) {
        //Don't want to bias sampling direction based on our chosen bxdf

        //sample_f and f of bxdf's done in tangent space
        Vector3f woT = worldToTangent * woW;
        Vector3f wiT(0.f);

        //this just for getting a wi
        color = bxdf_for_sampling->Sample_f_photon(woT, &wiT, xi, pdf, sampledType);
        *wiW = tangentToWorld * wiT;
    }
    float avepdf = 1.f;
    if( (bxdf_for_sampling->type & BSDF_SPECULAR) != BSDF_SPECULAR ) {
        //call BSDF::f to get sum of all BxDFs' f() function
        //try passing sampledtype here
        color = BSDF::f(woW, *wiW, type);

        //call BSDF::Pdf to get average pdf of all BxDFs
        avepdf = BSDF::Pdf(woW, *wiW, type);
    } else {
        avepdf /= filtered.size();
    }

    *pdf = avepdf;
    return color ;

}


Color3f BxDF::Sample_f_photon(const Vector3f &wo, Vector3f *wi, const Point2f &xi,
                       Float *pdf, BxDFType *sampledType) const
{
    *wi = WarpFunctions::squareToHemisphereUniform(xi);
    *pdf = Pdf(wo, *wi);
    return this->f(wo, *wi);
}
float BxDF::Pdf_photon(const Vector3f &wo, const Vector3f &wi) const
{
    return SameHemisphere(wo, wi) ? Inv2Pi : 0;
}
