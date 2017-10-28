#pragma once
//#include "geometry/primitive.h"
#include "scene/scene.h"
#include "samplers/sampler.h"
#include "warpfunctions.h"

// Forward declarations of structs used by our BVH tree
// They are defined in the .cpp file
class Scene;

typedef struct Photon {
    Photon() : wi(Point3f(),Vector3f()), objectHit(nullptr), worldnormal(), color(){}
    Photon(const Ray& incidentray, const Primitive* objhit, const Normal3f& n, const Color3f& pcolor)
        : wi(incidentray), objectHit(objhit), worldnormal(n), color(pcolor), children{nullptr, nullptr}, splitaxis(0) {}
    Ray wi;
    const Primitive* objectHit;
    Normal3f worldnormal;
    Color3f color;
    Photon *children[2];//0 is less, 1 is greater
    int splitaxis;
//    float tempsqrdDist;
} Photon;



enum PHOTONTYPE {DIRECT, INDIRECT, CAUSTIC};

class PhotonMap
{
public:
    PhotonMap();
    PhotonMap(Scene*);
    Bounds3f WorldBound() const;
    ~PhotonMap();

    Photon* recursivelyMakeNodes(const int start, const int end, std::vector<Photon>& photons, const Bounds3f& bounds, int* totalNodes);
    Bounds3f getBounds(const std::vector<Photon>& photons);
    void emitPhotons(std::shared_ptr<Light> mylight);
    void addPhoton(std::vector<Photon>& photons, const Intersection& isect, const Ray& woWray, const Color3f& photoncolor);
    void traceUntilNotSpecular(Intersection& isect, Color3f& photoncolor, Color3f& throughput, Ray& woWray, int& spechits, bool& continueflag);
    void kNearestNeighborsInPlane(const PHOTONTYPE type, std::vector<Photon*>& knearestphotons, const Intersection& isect, const float MAXSQRDDIST, const int MAXPHOTONS, const bool isSSSPM);
    void kNNInPlane(Photon* const p, float& worstdist, std::vector<Photon*>& knnphotons, const Intersection& isect, const float MAXSQRDDIST, const float MAXPHOTONS,const bool isSSSPM );

    //members
    std::vector<Photon> directphotons;//needed for SSS
    std::vector<Photon> indirectphotons;
    std::vector<Photon> causticphotons;
//    const float inv_targetphotonpower = 1.f / 0.006f;
//    const int maxbounces = 8;
//    const float photonpowerweight = 10;
//    const float artificialcausticscaling = 4;
    const float inv_targetphotonpower = 1.f / 0.001f;//0.001 was good//TODO: fix crash when 0.0008
    const int maxbounces = 10;
    const float photonpowerweight = 20;//20 was good
    const float artificialcausticscaling = 2;//2 is good for 0.001
   Scene* scene;
    Sampler* sampler;
    Photon* directnodes = nullptr;
    Photon* indirectnodes = nullptr;
    Photon* causticnodes = nullptr;
};
