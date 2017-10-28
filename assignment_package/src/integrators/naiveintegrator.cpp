#include "naiveintegrator.h"

Color3f NaiveIntegrator::Li(const Ray &ray, const Scene &scene, std::shared_ptr<Sampler> sampler, int depth) const
{    

    //TODO
    if(depth < 0){
        return Color3f(0.f);
    }

    Intersection isect = Intersection();
    if (scene.Intersect(ray,&isect)) {

        //everything in isect (point, t, b, n ) is in world space
        Vector3f woW = -1.f * ray.direction;
        Color3f color(0.f);

        if(isect.ProduceBSDF()) {//lights will return false
            Point2f xi = sampler->Get2D();
            float r = sampler->Get1D();
            Vector3f wiW(0.f);
            float pdf = 1.f;
            color = isect.bsdf->Sample_f(woW,&wiW,xi,r,&pdf);
            if(pdf != 0) {
                color /= pdf;
            }
            if(!IsBlack(color)){
                Ray wiWray = isect.SpawnRay(wiW);
                color *= AbsDot(wiW,isect.normalGeometric);
                Color3f recurseLi = Li(wiWray,scene,sampler,--depth);
                color *= recurseLi;
            }
        }
        //LTE
        return isect.Le(woW) + color;
    }
    return Color3f(0.f);

}
