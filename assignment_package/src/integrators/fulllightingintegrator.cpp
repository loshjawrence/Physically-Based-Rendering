#include "fulllightingintegrator.h"

//Color3f FullLightingIntegrator::Li(const Ray &ray, const Scene &scene, std::shared_ptr<Sampler> sampler, int depth, Color3f compoundedEnergy) const
//{
//    //TODO
//    if(depth <= recursionLimit - 3) {
//        float rand = sampler->Get1D();
//        float max = compoundedEnergy.r;
//        if ( max < compoundedEnergy.g) {
//            max = compoundedEnergy.g;
//        } else if ( max < compoundedEnergy.b) {
//            max = compoundedEnergy.b;
//        }
//        if(max < rand){
//            return Color3f(0.f);
//        }
//    }
//    Color3f finalcolor(0.f);
//    Color3f DLcolor(0.f);
//    Color3f naivecolor(0.f);

//    float DLpdf = 0.f;
//    float naivepdf = 0.f;
//    float BxdfPdf_for_DL = 0;
//    Float DLPdf_for_naive = 0;

//    Intersection isect = Intersection();
//    if (scene.Intersect(ray,&isect)) {
//        //everything in isect (point, t, b, n ) is in world space
//        Vector3f woW = -1.f * ray.direction;
//        Color3f emittedlightenergy = isect.Le(woW);
//        if (depth == 0 || isect.objectHit->GetMaterial() == nullptr) {
//            return emittedlightenergy;
//        }

//        if(isect.ProduceBSDF()) {
//            Point2f xi = sampler->Get2D();
//            Vector3f wiW_DL(0.f);

//            /*--------------------------------------------------
//             * call direct lighting returns lightengery/lightpdf
//             * ------------------------------------------------- */
//            float temp = xi.x * scene.lights.size();
//            xi.x = glm::clamp( (temp - std::floor(temp)) - 0.0001f, 0.f, 1.f) ;
//            uint32_t index = temp;

//            Color3f sampledlightenergy = scene.lights[index]->Sample_Li(isect,xi,&wiW_DL,&DLpdf);
//            DLpdf /= scene.lights.size();

//            Intersection closest = Intersection();
//            Ray wiWray_DL = isect.SpawnRay(wiW_DL);
//            scene.Intersect(wiWray_DL,&closest);

//            if(closest.objectHit == nullptr || closest.objectHit->areaLight != scene.lights.at(index)) {
//                sampledlightenergy = Color3f(0.f);
//            }
//            if (DLpdf > 0.f) {
//                Color3f bsdfresult_DL = isect.bsdf->f(woW,wiW_DL);
//                float absdot_DL = AbsDot(wiW_DL,isect.normalGeometric);
//                BxdfPdf_for_DL = isect.bsdf->Pdf(woW, wiW_DL);
//                float powerheuristic_DL = PowerHeuristic(1, DLpdf, 1, BxdfPdf_for_DL);
//                DLcolor = emittedlightenergy + (bsdfresult_DL * sampledlightenergy * absdot_DL * powerheuristic_DL)/DLpdf;
//            }


//            /*----------------------------------------------------------------
//            * bxdf part, borrowing from naive
//            * -----------------------------------------------------------------*/
//            xi = sampler->Get2D();
//            Vector3f wiW_naive(0.f);
//            float r = sampler->Get1D();
//            naivecolor = isect.bsdf->Sample_f(woW,&wiW_naive,xi,r,&naivepdf);
//            compoundedEnergy *= naivecolor;
//            if(naivepdf > 0) {
//                naivecolor /= naivepdf;
//            }
//            if(glm::length(naivecolor) > 0){
//                Ray wiWray = isect.SpawnRay(wiW_naive);
//                naivecolor *= AbsDot(wiW_naive,isect.normalGeometric);
//                Color3f recurseLi = Li(wiWray,scene,sampler,--depth,naivecolor);
//                naivecolor *= recurseLi;
//            }
//            DLPdf_for_naive = scene.lights[index]->Pdf_Li(isect,wiW_naive);
//            float powerheuristic_naive = PowerHeuristic(1, naivepdf, 1, DLPdf_for_naive);
//            naivecolor *= powerheuristic_naive;


//            finalcolor = DLcolor + naivecolor;
//        }
//        return finalcolor;
//    }
//    return finalcolor;
//}

Color3f FullLightingIntegrator::Li(const Ray &ray, const Scene &scene, std::shared_ptr<Sampler> sampler, int depth) const {
    Color3f accumcolor(0);
    Color3f throughput(1);
    bool specularBounce = false;
    Ray theray(ray.origin, ray.direction);

    for(int bounces = 0; bounces < depth; ++bounces){

        Intersection isect = Intersection();
        bool foundIntersection = scene.Intersect(theray, &isect);

        if(!foundIntersection) {
            break;
        }

        if(bounces == 0 || specularBounce) {
            accumcolor += throughput * isect.Le(-theray.direction);
        }
        if(!isect.ProduceBSDF()) {
            if(isect.objectHit->GetAreaLight()) {
                break;
            }
            theray = isect.SpawnRay(theray.direction);
            bounces--;
            continue;
        }
        if(isect.objectHit->GetMaterial() == nullptr) {
            break;
        }

        //compute direct
        Point2f xi = sampler->Get2D();
        Vector3f wiWdirect(0.f);
        Vector3f woW = -theray.direction;
        specularBounce = isect.bsdf->BxDFsMatchingFlags(BSDF_SPECULAR) > 0 ? true : false;
//        isect.objectHit->name;//break here to see what it does with glass
        //because bypassing all the light sampling stuff really affected glass

        float temp = xi.x * scene.lights.size();
        xi.x = glm::clamp( (temp - std::floor(temp)) - 0.0001f, 0.f, 1.f) ;
        uint32_t index = temp;
        float pdfdirect = 0;

        Color3f colordirect = scene.lights[index]->Sample_Li(isect,xi,&wiWdirect,&pdfdirect);
        pdfdirect /= scene.lights.size();

        Intersection closest = Intersection();
        Ray wiWray_direct = isect.SpawnRay(wiWdirect);
        scene.Intersect(wiWray_direct,&closest);

        if(pdfdirect <= 0.f || closest.objectHit == nullptr || closest.objectHit->areaLight != scene.lights.at(index)) {
            colordirect = Color3f(0.f);//occluded
        } else if (pdfdirect > 0.f) {
            Color3f bsdfdirect = isect.bsdf->f(woW,wiWdirect);
            float absdotdirect = AbsDot(wiWdirect,isect.normalGeometric);
            float BxdfPdf_for_direct = isect.bsdf->Pdf(woW, wiWdirect);
            float powerheuristic_direct = PowerHeuristic(1, pdfdirect, 1, BxdfPdf_for_direct);
            colordirect = (bsdfdirect * colordirect * absdotdirect * powerheuristic_direct)/pdfdirect;
        }

        //do the bxdf sample part for direct lighting
        Vector3f wiWdirectsample(0.f);
        float pdfdirectsample;
        Color3f colordirectsample = isect.bsdf->Sample_f(woW,&wiWdirectsample,sampler->Get2D(),sampler->Get1D(),&pdfdirectsample);
        if(glm::length(colordirectsample) > 0 && pdfdirectsample > 0) {
            float DLPdf_for_directsample = scene.lights[index]->Pdf_Li(isect,wiWdirectsample);

            if(DLPdf_for_directsample > 0) {//visible
                DLPdf_for_directsample /= (float)scene.lights.size();
                float powerheuristic_directsample = PowerHeuristic(1, pdfdirectsample, 1, DLPdf_for_directsample);
                colordirectsample *= AbsDot(wiWdirectsample,isect.normalGeometric);
                Ray wiWray = isect.SpawnRay(wiWdirectsample);
                Intersection lightisect;
                scene.Intersect(wiWray, &lightisect);
                Color3f Li = lightisect.Le(-wiWdirectsample);
                colordirectsample = (colordirectsample * Li * powerheuristic_directsample) / pdfdirectsample;
            } else {
                colordirectsample = Color3f(0);
            }
        } else {
            colordirectsample = Color3f(0);
        }


        //checking if this material is specular, direct wont contrib to specular
        //if bsdf->f and bsdf->pdf was setup correctly for specular materials then
        //shouldnt need to do this, it suggests that we can skip the light sampling
        //step (not the bxdf sampling step) for a spec bounce
        if(specularBounce) { colordirect = Color3f(0); }
        colordirect = colordirect + colordirectsample;

        accumcolor += throughput * colordirect;

        //get global illum ray from bxdf and loop
        Vector3f wiWbxdf;
        float pdfbxdf;
        Color3f colorbxdf = isect.bsdf->Sample_f(woW,&wiWbxdf,sampler->Get2D(),sampler->Get1D(),&pdfbxdf);

        Color3f current_throughput = colorbxdf * AbsDot(wiWbxdf, isect.normalGeometric) / pdfbxdf;
        if(glm::length2(colorbxdf) <= 0 || pdfbxdf <= 0) { current_throughput = Color3f(0); }

        throughput *= current_throughput;
        theray = isect.SpawnRay(wiWbxdf);

        if(bounces > 3) {
            float max = std::max(throughput.r, std::max(throughput.g, throughput.b));
            if(max < sampler->Get1D()) { break; }
            throughput /= max;
        }
    }
    return accumcolor;
}

float BalanceHeuristic(int nf, Float fPdf, int ng, Float gPdf)
{
    float denominator = (nf*fPdf + ng*gPdf);
    if(denominator < FLT_EPSILON) {
        return 0.f;
    }
    return (nf*fPdf) / denominator;
}

float PowerHeuristic(int nf, Float fPdf, int ng, Float gPdf)
{
    float f = nf * fPdf, g = ng * gPdf;

    float denominator = f*f + g*g;
    if(denominator < FLT_EPSILON) {
        return 0.f;
    }
    return (f*f) / (denominator);
}

