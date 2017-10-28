#include "photonmapintegrator.h"

const float SIGMA = 0.1;//good
const float SIGMASS = 0.5;
const float COEFF = 1.f/(sqrtf(2.f*3.14159265f)*SIGMA);
const float COEFFSS = 1.f/(sqrtf(2.f*3.14159265f)*SIGMASS);
const float ONE_OVER_DENOM = 1.f / (2.f*SIGMA*SIGMA);
const float ONE_OVER_DENOMSS = 1.f / (2.f*SIGMASS*SIGMASS);
const float EULERSNUM = 2.17828f;
const float NUMSIGMAS = 3.f;
const float NUMSIGMASSS = 6.f;
const float MAXSQRDDIST = (NUMSIGMAS*SIGMA)*(NUMSIGMAS*SIGMA);
const float MAXSQRDDISTSS = (NUMSIGMASSS*SIGMASS)*(NUMSIGMASSS*SIGMASS);
const int MAXPHOTONS = 500;
const float INDIRECTRAYS = 1;

//wiki equation
float gaussianWeight(const float rsquared) {
    return COEFF*powf(EULERSNUM, -rsquared*ONE_OVER_DENOM);
}
float gaussianWeightSS(const float rsquared) {
    return COEFFSS*powf(EULERSNUM, -rsquared*ONE_OVER_DENOMSS);
}


//gaussian filter page 33 of photonmappingjensensiggraph
const float alpha = 0.918f;
const float beta = 1.953f;
const float betaover2maxsqrddist = beta / (2.f*MAXSQRDDIST);
const float one_over_pdenom = 1.f / (1.f - powf(EULERSNUM, -beta));
__forceinline float gFilterPavicic90(const float rsquared) {
    return alpha * (1.f - (one_over_pdenom * ( 1.f - powf(EULERSNUM, -betaover2maxsqrddist*rsquared)) ));
}

Color3f PhotonMapIntegrator::Li(const Ray &ray, const Scene &scene, std::shared_ptr<Sampler> sampler, int depth) const {
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
//        specularBounce = (isect.bsdf->BxDFsMatchingFlags( (BxDFType)(BSDF_TRANSMISSION | BSDF_SPECULAR) ) > 0
//                        || isect.bsdf->BxDFsMatchingFlags( (BxDFType)(BSDF_REFLECTION | BSDF_SPECULAR) ) > 0) ? true : false;
        specularBounce = ( isect.bsdf->BxDFsMatchingFlags((BxDFType)(BSDF_SPECULAR)) > 0 ) ? true : false;

        float temp = xi.x * scene.lights.size();
        xi.x = glm::clamp( (temp - std::floor(temp)) - 0.0001f, 0.f, 1.f) ;
        uint32_t index = temp;
        float pdfdirect = 0;

        Color3f colordirect = scene.lights[index]->Sample_Li(isect,xi,&wiWdirect,&pdfdirect);
        pdfdirect /= scene.lights.size();

        Intersection closest = Intersection();
        Ray wiWray_direct = isect.SpawnRay(wiWdirect);
        scene.Intersect(wiWray_direct,&closest);

        if(closest.objectHit == nullptr || closest.objectHit->areaLight != scene.lights.at(index)) {
            colordirect = Color3f(0.f);//occluded
        } else if (pdfdirect > 0.f) {
            Color3f bsdfdirect = isect.bsdf->f(woW,wiWdirect);
            float absdotdirect = AbsDot(wiWdirect,isect.normalGeometric);
            float BxdfPdf_for_direct = isect.bsdf->Pdf(woW, wiWdirect);
            float powerheuristic_direct = PowerHeuristic(1, pdfdirect, 1, BxdfPdf_for_direct);
            colordirect = (bsdfdirect * colordirect * absdotdirect * powerheuristic_direct)/pdfdirect;
        } else {
            colordirect = Color3f(0);
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


        colordirect = colordirect + colordirectsample;

        //checking if this material is specular, direct wont contrib to specular
        if(specularBounce) { colordirect = Color3f(0); }

        if(!specularBounce) {//hit non-spec surface
            if (isect.bsdf->BxDFsMatchingFlags( (BxDFType)(BSDF_SSSPM) ) > 0 ) {
                return calcSSSPM(accumcolor, theray, woW, isect, scene, throughput, bounces, depth, sampler);
            } else {
                accumcolor += throughput * colordirect;
                return calcIndirectLightingFromPhotonMaps(accumcolor, woW, isect, scene, throughput, bounces, depth, sampler);
            }
        } else {
            accumcolor += throughput * colordirect;
        }

        //get global illum ray from bxdf and loop
        Vector3f wiWbxdf;
        float pdfbxdf;
        Color3f colorbxdf = isect.bsdf->Sample_f(woW,&wiWbxdf,sampler->Get2D(),sampler->Get1D(),&pdfbxdf);

        Color3f current_throughput = colorbxdf * AbsDot(wiWbxdf, isect.normalGeometric) / pdfbxdf;
        if(glm::length(colorbxdf) <= 0 || pdfbxdf <= 0) { current_throughput = Color3f(0); }

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


Color3f PhotonMapIntegrator::calcIndirectLightingFromPhotonMaps(Color3f& accumcolor,
       const Vector3f& woW, const Intersection& isect, const Scene& scene, const Color3f& throughput, const int bounces, const int depth, std::shared_ptr<Sampler> sampler) const {
    Vector3f wiWbxdf;
    float pdfbxdf;

    ///return if weak
    Color3f checkthroughput = throughput;
    Color3f colorbxdf = isect.bsdf->Sample_f(woW,&wiWbxdf,sampler->Get2D(),sampler->Get1D(),&pdfbxdf);

    Color3f current_throughput = colorbxdf * AbsDot(wiWbxdf, isect.normalGeometric) / pdfbxdf;
    if(glm::length(colorbxdf) <= 0 || pdfbxdf <= 0) {
        current_throughput = Color3f(0);
    }

    checkthroughput *= current_throughput;
    Ray theray = isect.SpawnRay(wiWbxdf);

    if(bounces > 3) {
        float max = std::max(checkthroughput.r, std::max(checkthroughput.g, checkthroughput.b));
        if(max < sampler->Get1D()) { return accumcolor; }
    }

    if(bounces+1 >= depth) { return accumcolor; }
    ///end weak check

    //sample caustic map at the current nonspec intersection

    Color3f totalcausticphotonsum = integratePhotonMap(CAUSTIC, woW, isect,false);
    totalcausticphotonsum *= throughput;

    //Shoot rays from our nonspec isect and integrate indirect photon map at nonspec surfaces(or continue until hit nonspec and integrate)
    //don't update throughput for each ray
    Intersection indisect = Intersection();

    Color3f totalindphotonsum(0.f);
    for(int i = 0; i < INDIRECTRAYS; ++i) {
        Color3f indthroughput = throughput;
        Color3f pickupfromaccumcolor = accumcolor + totalcausticphotonsum;
        Color3f colorbxdf = isect.bsdf->Sample_f(woW,&wiWbxdf,sampler->Get2D(),sampler->Get1D(),&pdfbxdf);

        Color3f current_throughput = colorbxdf * AbsDot(wiWbxdf, isect.normalGeometric) / pdfbxdf;
        if(glm::length(colorbxdf) <= 0 || pdfbxdf <= 0) {
            current_throughput = Color3f(0);
        }

        indthroughput *= current_throughput;
        Ray theray = isect.SpawnRay(wiWbxdf);
        if( !(scene.Intersect(theray,&indisect)) ) { continue; }//return pickupfromaccumcolor; }
        if( !(indisect.ProduceBSDF()) ) { continue; }//return pickupfromaccumcolor; }
        //continue until not spec
        bool continueflag = false;
        int spechits = 0;
        traceUntilNotSpecular(indisect, pickupfromaccumcolor, indthroughput, theray,spechits,continueflag);
        if(continueflag || spechits+bounces >= depth) { continue; }//return pickupfromaccumcolor; }

        //sum up f(wi,wo) of all photons found
        Color3f indphotonsum = integratePhotonMap(INDIRECT, -theray.direction, indisect,false);
        indphotonsum *= indthroughput;
        totalindphotonsum += indphotonsum;
    }
    totalindphotonsum = totalindphotonsum * (1.f/INDIRECTRAYS);//ave

    return accumcolor + totalindphotonsum + totalcausticphotonsum;
}

//integral given on page 33 of photonmappingjensensiggraph
Color3f PhotonMapIntegrator::integratePhotonMap(const PHOTONTYPE photontype, const Vector3f& woW, const Intersection& isect, const bool isSSSPM) const {
    Color3f color(0.f);

    std::vector<Photon*> knearestphotons;
    scene->pm->kNearestNeighborsInPlane(photontype, knearestphotons, isect, MAXSQRDDIST, MAXPHOTONS,isSSSPM);

    const int size = knearestphotons.size();
    for(int i = 0; i < size; ++i) {
        const Photon* const p = knearestphotons[i];
        Vector3f wiW = p->wi.direction;
        Color3f photoncolor = p->color;
        Color3f bsdfresult = isect.bsdf->f(woW,wiW);
        float absdot = AbsDot(wiW,isect.normalGeometric);
        //no dividing by pi*maxsqrddist if using gaussian weights
        color += bsdfresult * photoncolor * absdot * gaussianWeight(glm::distance2(p->wi.origin,isect.point));
//        color += bsdfresult * photoncolor * absdot * gFilterPavcic90(glm::distance2(p->wi.origin,isect.point));
    }

    return color;
}


void PhotonMapIntegrator::traceUntilNotSpecular(Intersection& isect, Color3f& accumcolor, Color3f& throughput, Ray& woWray, int& spechits, bool& continueflag) const {
//    while(   isect.bsdf->BxDFsMatchingFlags( (BxDFType)(BSDF_TRANSMISSION | BSDF_SPECULAR)) > 0
//          || isect.bsdf->BxDFsMatchingFlags( (BxDFType)(BSDF_REFLECTION   | BSDF_SPECULAR)) > 0) {
    while(   isect.bsdf->BxDFsMatchingFlags( (BxDFType)(BSDF_SPECULAR)) > 0) {
        spechits++;
        Vector3f wiW = -1.f * woWray.direction;
        Point2f xi = sampler->Get2D();
        float r = sampler->Get1D();
        Vector3f woW;
        float pdf = 1.f;
        Color3f color = isect.bsdf->Sample_f(wiW,&woW,xi,r,&pdf);
        if(pdf == 0 || glm::length(color) <= 0.f) {
            continueflag = true;
            break;
        }
        //GOOD CHANCE THIS IS WRONG
//        accumcolor *= throughput;
        Color3f current_throughput = color * AbsDot(woW, isect.normalGeometric);// * (1.f / pdf);
        throughput *= current_throughput;
        accumcolor *= throughput;
        woWray = isect.SpawnRay(woW);
        if( !(scene->Intersect(woWray,&isect)) ) { continueflag = true; break; }
        if( !(isect.ProduceBSDF()) ) { continueflag = true; break; }
    }
}


Color3f PhotonMapIntegrator::calcSSSPM(Color3f& accumcolor, const Ray& theincray,
       const Vector3f& woW, const Intersection& isect, const Scene& scene, const Color3f& throughput, const int bounces, const int depth, std::shared_ptr<Sampler> sampler) const {
    Vector3f wiWbxdf;
    float pdfbxdf;

    ///return if weak
    Color3f checkthroughput = throughput;
    Color3f colorbxdf = isect.bsdf->Sample_f(woW,&wiWbxdf,sampler->Get2D(),sampler->Get1D(),&pdfbxdf);

    Color3f current_throughput = colorbxdf * AbsDot(wiWbxdf, isect.normalGeometric) / pdfbxdf;
    if(glm::length(colorbxdf) <= 0 || pdfbxdf <= 0) {
        current_throughput = Color3f(0);
    }

    checkthroughput *= current_throughput;
//    Ray theray = isect.SpawnRay(wiWbxdf);

    if(bounces > 3) {
        float max = std::max(checkthroughput.r, std::max(checkthroughput.g, checkthroughput.b));
        if(max < sampler->Get1D()) { return accumcolor; }
    }

    if(bounces+1 >= depth) { return accumcolor; }
    ///end weak check

    //sample caustic map at the current nonspec intersection

    Color3f totalcausticphotonsum = integratePhotonMap(CAUSTIC, woW, isect,1);
    totalcausticphotonsum *= throughput;



    //Grab photons in vincinity
//    std::vector<Photon*> nearbypoints;
//    scene.pm->kNearestNeighborsInPlane(DIRECT, nearbypoints, isect, MAXSQRDDISTSS, MAXPHOTONS,true);
    //grab subset,montecarlo integration
    std::vector<Photon*> nearbypointsall;
    scene.pm->kNearestNeighborsInPlane(DIRECT, nearbypointsall, isect, MAXSQRDDISTSS, MAXPHOTONS,true);

    Color3f accumtotalindphotonsum(0.f);
    float gwsum = 0;
//    for(int i = 0; i < nearbypoints.size(); ++i) {
    {
        int rand = sqrtf(sampler->Get1D()) * nearbypointsall.size();
        Photon* p = nearbypointsall[rand];
        Intersection sourceisect = isect;
//        Photon* p = nearbypoints[i];
        sourceisect.point = p->wi.origin;
        sourceisect.normalGeometric = p->worldnormal;

        //compute direct
        Point2f xi = sampler->Get2D();
        Vector3f wiWdirect(0.f);
//        Vector3f woWdirectgwintegration = -theincray.direction;
        Vector3f woWdirectgwintegration = glm::normalize(isect.point - sourceisect.point);

        float temp = xi.x * scene.lights.size();
        xi.x = glm::clamp( (temp - std::floor(temp)) - 0.0001f, 0.f, 1.f) ;
        uint32_t index = temp;
        float pdfdirect = 0;

        Color3f colordirect = scene.lights[index]->Sample_Li(sourceisect,xi,&wiWdirect,&pdfdirect);
        pdfdirect /= scene.lights.size();

        Intersection closest = Intersection();
        Ray wiWray_direct = sourceisect.SpawnRay(wiWdirect);
        scene.Intersect(wiWray_direct,&closest);

        if(closest.objectHit == nullptr || closest.objectHit->areaLight != scene.lights.at(index)) {
            colordirect = Color3f(0.f);//occluded
        } else if (pdfdirect > 0.f) {
            Color3f bsdfdirect = sourceisect.bsdf->f(woWdirectgwintegration,wiWdirect);
            float absdotdirect = AbsDot(wiWdirect,sourceisect.normalGeometric);
            float BxdfPdf_for_direct = sourceisect.bsdf->Pdf(woWdirectgwintegration, wiWdirect);
            float powerheuristic_direct = PowerHeuristic(1, pdfdirect, 1, BxdfPdf_for_direct);
            colordirect = (bsdfdirect * colordirect * absdotdirect * powerheuristic_direct)/pdfdirect;
        } else {
            colordirect = Color3f(0);
        }

        //do the bxdf sample part for direct lighting
        Vector3f wiWdirectsample(0.f);
        float pdfdirectsample;
        Color3f colordirectsample = sourceisect.bsdf->Sample_f(woWdirectgwintegration,&wiWdirectsample,sampler->Get2D(),sampler->Get1D(),&pdfdirectsample);
        if(glm::length(colordirectsample) > 0 && pdfdirectsample > 0) {
            float DLPdf_for_directsample = scene.lights[index]->Pdf_Li(sourceisect,wiWdirectsample);

            if(DLPdf_for_directsample > 0) {//visible
                DLPdf_for_directsample /= (float)scene.lights.size();
                float powerheuristic_directsample = PowerHeuristic(1, pdfdirectsample, 1, DLPdf_for_directsample);
                colordirectsample *= AbsDot(wiWdirectsample,sourceisect.normalGeometric);
                Ray wiWray = sourceisect.SpawnRay(wiWdirectsample);
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

        //DIRECT LIGHTING CALC FINISHED
        colordirect = colordirect + colordirectsample;
        const float gw = gaussianWeightSS(glm::distance2(p->wi.origin, isect.point));
        gwsum += gw;
        Color3f amounttoadd = (throughput * colordirect * gw);
        accumcolor += amounttoadd;


        //INDIRECT LIGHTING FOR THIS DIRECT SOURCE
        Intersection indisect = Intersection();
        Color3f totalindphotonsum(0.f);
        for(int i = 0; i < INDIRECTRAYS; ++i) {
            Color3f indthroughput = throughput;
            Color3f pickupfromaccumcolor = accumcolor + totalcausticphotonsum;
            Color3f colorbxdf = sourceisect.bsdf->Sample_f(woW,&wiWbxdf,sampler->Get2D(),sampler->Get1D(),&pdfbxdf);

            Color3f current_throughput = colorbxdf * AbsDot(wiWbxdf, sourceisect.normalGeometric) / pdfbxdf;
            if(glm::length(colorbxdf) <= 0 || pdfbxdf <= 0) {
                current_throughput = Color3f(0);
            }

            indthroughput *= current_throughput;
            Ray theray = sourceisect.SpawnRay(wiWbxdf);
            if( !(scene.Intersect(theray,&indisect)) ) { continue; }//return pickupfromaccumcolor; }
            if( !(indisect.ProduceBSDF()) ) { continue; }//return pickupfromaccumcolor; }
            //continue until not spec
            bool continueflag = false;
            int spechits = 0;
            traceUntilNotSpecular(indisect, pickupfromaccumcolor, indthroughput, theray,spechits,continueflag);
            if(continueflag || spechits+bounces >= depth) { continue; }//return pickupfromaccumcolor; }

            //sum up f(wi,wo) of all photons found
            Color3f indphotonsum = integratePhotonMap(INDIRECT, -theray.direction, indisect, false);
            indphotonsum *= indthroughput;
            totalindphotonsum += indphotonsum;
        }
        totalindphotonsum = totalindphotonsum * (1.f/INDIRECTRAYS);//ave
        totalindphotonsum *= gw;
        accumtotalindphotonsum += totalindphotonsum;
    }

    //was diviing by nearbypoints.size() or gwsum
//    accumcolor =   (accumcolor  + accumtotalindphotonsum)*(1.f/nearbypoints.size());
    accumcolor =   (accumcolor  + accumtotalindphotonsum);


    //INDIRECT LIGHTING (INDIRECT DIFFUSION)
    std::vector<Photon*> nearbypointsINDall;
    scene.pm->kNearestNeighborsInPlane(INDIRECT, nearbypointsINDall, isect, MAXSQRDDISTSS, MAXPHOTONS,true);
    Intersection indisect = Intersection();
    Color3f totalindphotonsum(0.f);
    for(int i = 0; i < INDIRECTRAYS; ++i) {
        Color3f indthroughput = throughput;
        Color3f pickupfromaccumcolor = accumcolor + totalcausticphotonsum;

        //pick random indirect photon on surface
        Intersection sourceisectIND = isect;
        if(nearbypointsINDall.size() > 0) {
            int rand = sqrtf(sampler->Get1D()) * nearbypointsINDall.size();
            Photon* p = nearbypointsINDall[rand];
            sourceisectIND.point = p->wi.origin;
            sourceisectIND.normalGeometric = p->worldnormal;
        }

//        Color3f colorbxdf = sourceisectIND.bsdf->Sample_f(woW,&wiWbxdf,sampler->Get2D(),sampler->Get1D(),&pdfbxdf);

        //NEWWAY: should woW be the direction to the sourceisect from origisect?
        Vector3f woWindirectgwintegration = glm::normalize(isect.point - sourceisectIND.point);
        Color3f colorbxdf = sourceisectIND.bsdf->Sample_f(woWindirectgwintegration,&wiWbxdf,sampler->Get2D(),sampler->Get1D(),&pdfbxdf);

        Color3f current_throughput = colorbxdf * AbsDot(wiWbxdf, sourceisectIND.normalGeometric) / pdfbxdf;
        if(glm::length(colorbxdf) <= 0 || pdfbxdf <= 0) {
            current_throughput = Color3f(0);
        }

        indthroughput *= current_throughput;
        Ray theray = sourceisectIND.SpawnRay(wiWbxdf);
        if( !(scene.Intersect(theray,&indisect)) ) { continue; }//return pickupfromaccumcolor; }
        if( !(indisect.ProduceBSDF()) ) { continue; }//return pickupfromaccumcolor; }
        //continue until not spec
        bool continueflag = false;
        int spechits = 0;
        traceUntilNotSpecular(indisect, pickupfromaccumcolor, indthroughput, theray,spechits,continueflag);
        if(continueflag || spechits+bounces >= depth) { continue; }//return pickupfromaccumcolor; }

        //sum up f(wi,wo) of all photons found
        Color3f indphotonsum = integratePhotonMap(INDIRECT, -theray.direction, indisect,false);
        indphotonsum *= indthroughput;
        totalindphotonsum += indphotonsum;
    }
    totalindphotonsum = totalindphotonsum * (1.f/INDIRECTRAYS);//ave

    return accumcolor + totalindphotonsum + totalcausticphotonsum;
}

float PhotonMapIntegrator::BalanceHeuristic(int nf, Float fPdf, int ng, Float gPdf) const {
    float denominator = (nf*fPdf + ng*gPdf);
    if(denominator < FLT_EPSILON) {
        return 0.f;
    }
    return (nf*fPdf) / denominator;
}

float PhotonMapIntegrator::PowerHeuristic(int nf, Float fPdf, int ng, Float gPdf) const {
    float f = nf * fPdf, g = ng * gPdf;

    float denominator = f*f + g*g;
    if(denominator < FLT_EPSILON) {
        return 0.f;
    }
    return (f*f) / (denominator);
}
