#include "directlightingintegrator.h"

Color3f DirectLightingIntegrator::Li(const Ray &ray, const Scene &scene, std::shared_ptr<Sampler> sampler, int depth) const
{
    //TODO
    Color3f color(0.f);

    Intersection isect = Intersection();
    if (scene.Intersect(ray,&isect)) {
        //everything in isect (point, t, b, n ) is in world space
        Vector3f woW = -1.f * ray.direction;
        Color3f emittedlightenergy = isect.Le(woW);
        if (depth == 0 || isect.objectHit->GetMaterial() == nullptr) {
            return emittedlightenergy;
        }

        if(isect.ProduceBSDF()) {
            Point2f xi = sampler->Get2D();
            Vector3f wiW(0.f);
            float pdf = 0;

            float temp = xi.x * scene.lights.size();
            xi.x = glm::clamp( (temp - std::floor(temp)) - 0.0001f, 0.f, 1.f) ;
            uint32_t index = temp;

            Color3f sampledlightenergy = scene.lights[index]->Sample_Li(isect,xi,&wiW,&pdf);
            if (pdf == 0.f) {
                return Color3f(0.f);
            }

            pdf /= scene.lights.size();
            sampledlightenergy /= pdf;

            Intersection closest = Intersection();
            Ray wiWray = isect.SpawnRay(wiW);
            scene.Intersect(wiWray,&closest);

            if(closest.objectHit == nullptr || closest.objectHit->areaLight != scene.lights.at(index)) {
                sampledlightenergy = Color3f(0.f);
            }

            Color3f bsdfresult = isect.bsdf->f(woW,wiW);
            float absdot = std::fabs(glm::dot(wiW,isect.normalGeometric));

            color = emittedlightenergy + bsdfresult * sampledlightenergy * absdot;
        } else {
            //no material attached to isect, color still 0
        }
    }

    return color;

}
