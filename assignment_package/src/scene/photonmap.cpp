#include "photonmap.h"


PhotonMap::~PhotonMap(){
    if(directnodes != nullptr){
        directnodes = nullptr;
    }

    if(indirectnodes != nullptr){
        indirectnodes = nullptr;
    }

    if(causticnodes != nullptr){
        causticnodes = nullptr;
    }

    if(sampler != nullptr){
        delete sampler;
        sampler = nullptr;
    }
}

PhotonMap::PhotonMap(Scene* thescene)
    : scene(thescene)
{
    const int numlights = scene->lights.size();
    if(numlights == 0) {
        return;
    }

    //emit photons from each light
    for(int i = 0; i < numlights; ++i) {
        std::shared_ptr<Light> mylight = scene->lights[i];
        emitPhotons(mylight);
    }


    //build direct kd tree
    int totalDirectNodes = 0;
    directphotons.shrink_to_fit();
    std::cout << "\nNum direct photons: " << directphotons.size();
    if(directphotons.size() > 0) {
        Bounds3f directbounds = getBounds(directphotons);
        directnodes = recursivelyMakeNodes(0, directphotons.size(), directphotons, directbounds, &totalDirectNodes);
    }
    std::cout << "\nNum direct kd nodes: " << totalDirectNodes;

    //build indirect kd tree
    int totalIndirectNodes = 0;
    indirectphotons.shrink_to_fit();
    std::cout << "\nNum indirect photons: " << indirectphotons.size();
    if(indirectphotons.size() > 0) {
        Bounds3f indirectbounds = getBounds(indirectphotons);
        indirectnodes = recursivelyMakeNodes(0, indirectphotons.size(), indirectphotons, indirectbounds, &totalIndirectNodes);
    }
    std::cout << "\nNum indirect kd nodes: " << totalIndirectNodes;

    //build caustic kd tree
    int totalCausticNodes = 0;
    causticphotons.shrink_to_fit();
    if(causticphotons.size() > 0) {
        Bounds3f causticbounds = getBounds(causticphotons);
        causticnodes = recursivelyMakeNodes(0, causticphotons.size(), causticphotons, causticbounds, &totalCausticNodes);
    }
    std::cout << "\nNum caustic photons: " << causticphotons.size() << "\nNum caustic nodes: " << totalCausticNodes;
}

Bounds3f PhotonMap::getBounds(const std::vector<Photon> &photons) {
    const Point3f& firstpoint = photons[0].wi.origin;
    float xmin = firstpoint.x;
    float ymin = firstpoint.y;
    float zmin = firstpoint.z;
    float xmax = firstpoint.x;
    float ymax = firstpoint.y;
    float zmax = firstpoint.z;
    for(int i = 1; i < photons.size(); ++i){
        const Point3f& point = photons[i].wi.origin;
        if(point.x < xmin) {
            xmin = point.x;
        } else if( point.x > xmax) {
            xmax = point.x;
        }

        if(point.y < ymin) {
            ymin = point.y;
        } else if( point.y > ymax) {
            ymax = point.y;
        }

        if(point.z < zmin) {
            zmin = point.z;
        } else if( point.z > zmax) {
            zmax = point.z;
        }

    }
    const float bias = 0.005;
    return Bounds3f(Point3f(xmin-bias,ymin-bias,zmin-bias), Point3f(xmax+bias,ymax+bias,zmax+bias));
}

void PhotonMap::emitPhotons(std::shared_ptr<Light> mylight) {
    //add loop for tracing ray until it dies from russian roullllette
    //add flag for caustic if first intersection is specular

    //proxywattage is luminance times intensity times area
    const int numphotons = mylight->getProxyWattage() * inv_targetphotonpower;
    const Color3f photoncolor = photonpowerweight * ( mylight->getEmittedLight() * mylight->getLightArea() ) * (1.f/numphotons);

    //sampler
    sampler = new Sampler(numphotons,numphotons);//use stratified random samples to get light's emission ray
    std::vector<Point2f> SRsamples = sampler->GenerateStratifiedSamples();

    std::cout << "\nNum photons shot from light \"" << mylight->name.toStdString() << "\": " << numphotons;
    std::cout << "\nEach photon color: " << photoncolor.r << ", " << photoncolor.g << ", " << photoncolor.b;
    for(int i = 0; i < numphotons; ++i) {
        //get worldspace ray from stratified random sampling of suface point and direction
        Ray emissionray = Ray(Point3f(), Vector3f());
        float emissionpdf;
        mylight->getSurfaceEmissionRay(SRsamples[i],sampler->Get2D(),emissionray,emissionpdf);
        Intersection isect = Intersection();
        Color3f throughput(1);
        int bounces = 0;
        bool continueflag = false;
        Ray woWray = emissionray;
        Color3f myphotoncolor = photoncolor;

        //factor into tracUntil =============FROM HERE
        if( !(scene->Intersect(woWray,&isect)) ) { continue; }
        if( !(isect.ProduceBSDF()) ) { continue; }

        traceUntilNotSpecular(isect, myphotoncolor, throughput, woWray, bounces, continueflag);
        if(continueflag) { continue; }

        //HIT DIFFUSE SURFACE
        if(bounces > 0) {//store in caustic map if we hit specular first
            Color3f causticcolor = myphotoncolor*artificialcausticscaling;
            addPhoton(causticphotons, isect, woWray, causticcolor);
        } else {
            //first bounce, add direct lighting photons for SSS
            addPhoton(directphotons, isect, woWray, myphotoncolor);
        }
        //==============TO HERE

        while(bounces < maxbounces){
            bounces++;
            //get new ray from diffuse surface sample f
            Vector3f wiW = -1.f * woWray.direction;
            Point2f xi = sampler->Get2D();
            float r = sampler->Get1D();
            Vector3f woW;
            float pdf = 1.f;
            Color3f color = isect.bsdf->Sample_f(wiW,&woW,xi,r,&pdf);
            if(pdf == 0 || glm::length2(color) <= 0.f) {
                continueflag = true;
                break;
            }

//            myphotoncolor *= throughput;
            Color3f current_throughput = color * AbsDot(woW, isect.normalGeometric) *(1.f / pdf);
            throughput *= current_throughput;
            myphotoncolor *= throughput;

            //trace into scene
            woWray = isect.SpawnRay(woW);
            if( !(scene->Intersect(woWray,&isect)) ) { break; }
            if( !(isect.ProduceBSDF()) ) { break; }

            //traceUntilNotSpecular
            traceUntilNotSpecular(isect, myphotoncolor, throughput, woWray, bounces, continueflag);
            if(continueflag) { break; }

            //HIT DIFFUSE SURFACE
            addPhoton(indirectphotons, isect, woWray, myphotoncolor);

            if(bounces > 3) {
                float max = std::max(throughput.r, std::max(throughput.g, throughput.b));
                if(max < sampler->Get1D()) { break; }
                throughput /= max;
            }
        }
    }
    delete sampler;
}

void PhotonMap::traceUntilNotSpecular(Intersection& isect, Color3f& photoncolor, Color3f& throughput, Ray& woWray, int& spechits, bool& continueflag) {
//    while(   isect.bsdf->BxDFsMatchingFlags( (BxDFType)(BSDF_TRANSMISSION | BSDF_SPECULAR)) > 0
//          || isect.bsdf->BxDFsMatchingFlags( (BxDFType)(BSDF_REFLECTION   | BSDF_SPECULAR)) > 0) {
    while( isect.bsdf->BxDFsMatchingFlags(BSDF_SPECULAR) > 0 ) {
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
//        photoncolor *= throughput;
        Color3f current_throughput = color * AbsDot(woW, isect.normalGeometric);// * (1.f / pdf);
        throughput *= current_throughput;
        photoncolor *= throughput;
        woWray = isect.SpawnRay(woW);
        if( !(scene->Intersect(woWray,&isect)) ) { continueflag = true; break; }
        if( !(isect.ProduceBSDF()) ) { continueflag = true; break; }
    }
}

void PhotonMap::addPhoton(std::vector<Photon>& photons, const Intersection& isect, const Ray& woWray, const Color3f& photoncolor) {
    Ray myray = Ray(isect.point, -1.f * woWray.direction);
    photons.push_back(Photon(myray, isect.objectHit, isect.normalGeometric, photoncolor));
}

Photon* PhotonMap::recursivelyMakeNodes(const int start, const int end, std::vector<Photon>& photons, const Bounds3f& bounds, int* totalNodes) {
//end is 1 past the elements we care about
//example end conditions
//0 1 2 3
//^
//start
//  ^ mid
//    ^ mid+1
//      ^ end

//0 1 2
//^
//start and mid (needs start == end check)
//  ^ mid+1
//    ^ end
    if(start == end-1) {
        (*totalNodes)++;
        return &(photons[start]);
    } else if ( start == end ) {
        return nullptr;//case where mid
    }

    //pick largest axis span, x,y,z = 0,1,2
    const int dimtosplit = bounds.MaximumExtent();

    const int mid = (start+end-1)/2;
    std::nth_element( &(photons[start]), &(photons[mid]), &photons[end-1]+1,
            [dimtosplit](const Photon& p1, const Photon& p2) {
        return p1.wi.origin[dimtosplit] < p2.wi.origin[dimtosplit];
    });


    Photon* const newnode = &(photons[mid]);
    newnode->splitaxis = dimtosplit;
    Point3f leftkdpartition  = bounds.max;
    Point3f rightkdpartition = bounds.min;
    const float splitpoint = photons[mid].wi.origin[dimtosplit];
    leftkdpartition[dimtosplit]  = splitpoint;
    rightkdpartition[dimtosplit] = splitpoint;

    newnode->children[0] = recursivelyMakeNodes(start, mid, photons, Bounds3f(bounds.min,       leftkdpartition),  totalNodes);
    newnode->children[1] = recursivelyMakeNodes(mid+1, end, photons, Bounds3f(rightkdpartition, bounds.max)     , totalNodes);
    (*totalNodes)++;
    return newnode;
}

void PhotonMap::kNearestNeighborsInPlane(const PHOTONTYPE type, std::vector<Photon*>& knnphotons, const Intersection& isect, const float MAXSQRDDIST, const int MAXPHOTONS, const bool isSSSPM){
    //wrapper call to the actual kNNInPlane recursive function
    float worstdist = -FLT_MAX;
    float maxsqrddist = MAXSQRDDIST;
    float maxphotons = MAXPHOTONS;
//    if(isSSSPM) {
//        maxsqrddist *= 1.f;
//        maxphotons *= 1.f;
//    }
    if(type == INDIRECT) {
        kNNInPlane(indirectnodes, worstdist, knnphotons, isect, maxsqrddist, maxphotons, isSSSPM);
    } else if (type == DIRECT){
        kNNInPlane(directnodes , worstdist, knnphotons, isect, maxsqrddist, maxphotons, isSSSPM);
    } else if (type == CAUSTIC){
        kNNInPlane(causticnodes , worstdist, knnphotons, isect, maxsqrddist, maxphotons, isSSSPM);
    }
}

//https://www.youtube.com/watch?v=dyU2WyjhyLM
//wiki:A branch is only eliminated when k points have been found and the branch cannot have points closer than any of the k current bests.
//worstdist is the distance2 of the furtherest point found so far within the search range
__forceinline bool samePlane(const Point3f& p, const Intersection& isect) {
    return (AbsDot(glm::normalize(p - isect.point), isect.normalGeometric) < 0.005);
}

void insertInOrder(std::vector<Photon*>& knnphotons, Photon* const p, const Intersection& isect, bool pushback, const int MAXPHOTONS) {

    const int size = knnphotons.size();

    if(pushback) { knnphotons.push_back(p); return; }

    for(int i = 0; i < size; ++i) {
//        if(p->tempsqrdDist < knnphotons[i]->tempsqrdDist) {
        if(glm::distance2(p->wi.origin, isect.point) < glm::distance2(knnphotons[i]->wi.origin, isect.point)) {
            auto it = knnphotons.begin()+i;
            knnphotons.insert(it, p);
            break;
        }
    }

    if(knnphotons.size() > MAXPHOTONS) { knnphotons.pop_back(); }
}

void possiblyAddPhoton(Photon* const p, float& worstdist, std::vector<Photon*>& knnphotons, const Intersection& isect, const float MAXSQRDDIST, const float MAXPHOTONS, const bool isSSSPM) {
    const Point3f X = isect.point;
    const Point3f ppoint = p->wi.origin;
    if(isSSSPM && p->objectHit != isect.objectHit) { return; }
    if(samePlane(ppoint, isect) || isSSSPM) {
        const float dist2 = glm::distance2(X,ppoint);
        const bool full = (knnphotons.size() == MAXPHOTONS);
        if( (!full && dist2 < MAXSQRDDIST) || (full && dist2 < worstdist)) {
            bool pushback = false;
            if(dist2 > worstdist) { worstdist = dist2; pushback = true;}
            insertInOrder(knnphotons,p,isect,pushback,MAXPHOTONS);
        }
    }
}

void PhotonMap::kNNInPlane(Photon* const p, float& worstdist, std::vector<Photon*>& knnphotons, const Intersection& isect, const float MAXSQRDDIST, const float MAXPHOTONS, const bool isSSSPM ) {
    if(p == nullptr) { return; }
    const Point3f X = isect.point;
    const Point3f ppoint = p->wi.origin;

    //if leaf, try to add photon
    if(p->children[0] == nullptr && p->children[1] == nullptr) {
        possiblyAddPhoton(p, worstdist, knnphotons, isect, MAXSQRDDIST, MAXPHOTONS, isSSSPM);
        return;
    }

    //DFS, drill down to first leaf node
    const int axis = p->splitaxis;
    int otherchild = 0;
    if(X[axis] < ppoint[axis]){
        kNNInPlane(p->children[0], worstdist, knnphotons, isect, MAXSQRDDIST, MAXPHOTONS, isSSSPM);
        otherchild = 1;
    } else {
        kNNInPlane(p->children[1], worstdist, knnphotons, isect, MAXSQRDDIST, MAXPHOTONS, isSSSPM);
    }

    //check this node
    possiblyAddPhoton(p, worstdist, knnphotons, isect, MAXSQRDDIST, MAXPHOTONS, isSSSPM);

    //call knn on otherchild if (ppoint[axis]-X[axis])^2 < maxsqrddist or worstdist depending on whether we're full or not
    //basically checking if there's a photon in that subtree that can be added
    const bool full = (knnphotons.size() == MAXPHOTONS);
    float dist2toaxis = ppoint[axis] - X[axis];
    dist2toaxis *= dist2toaxis;
    if( (!full && dist2toaxis < MAXSQRDDIST) || (full && dist2toaxis < worstdist)) {
        kNNInPlane(p->children[otherchild], worstdist, knnphotons, isect, MAXSQRDDIST, MAXPHOTONS, isSSSPM);
    }
}
