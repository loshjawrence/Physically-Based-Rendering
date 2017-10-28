#include "implicitsurface.h"
#include <warpfunctions.h>


#define SURFACE 0
Point2f ImplicitSurface::GetUVCoordinates(const Point3f &point) const {
//   return Point2f(point.x/transform.getScale().x,point.y/transform.getScale().y);
    Point3f p = glm::normalize(point);
    float phi = atan2f(p.z, p.x);
    if(phi < 0)
    {
        phi += TwoPi;
    }
    float theta = glm::acos(p.y);
    return Point2f(1 - phi/TwoPi, 1 - theta / Pi);
}
float ImplicitSurface::Area() const{
    return 1;
}
Intersection ImplicitSurface::Sample(const Point2f &xi, Float *pdf) const
{
    //using sphere's sample
    Point3f pObj = WarpFunctions::squareToSphereUniform(xi);

    Intersection it;
    it.normalGeometric = glm::normalize( transform.invTransT() *pObj );
    it.point = Point3f(transform.T() * glm::vec4(pObj.x, pObj.y, pObj.z, 1.0f));

    *pdf = 1.0f / Area();

    return it;
}

void ImplicitSurface::getSurfaceEmissionRay(const Point2f &xi, const Point2f& xk, Ray& ray ,float& pdf) const {
    //could sample surface using random direction, getting all surface points along direction and picking one
}
float signedDistFunc(const Point3f& p) {
    const float x = p.x;
    const float y = p.y;
    const float z = p.z;
    const float x2 = x*x;
    const float y2 = y*y;
    const float z2 = z*z;
#if SURFACE == 0 //tanglecube
    //http://mathworld.wolfram.com/Tanglecube.html
    const float x4 = x2*x2;
    const float y4 = y2*y2;
    const float z4 = z2*z2;
    return x4 - 5*x2 + y4 -5*y2 + z4 - 5*z2 + 11.8;
#elif SURFACE == 1//chair surface
    //http://mathworld.wolfram.com/ChairSurface.html
    const float k = 5;
    const float a = 0.95;
    const float b = 0.8;
    const float term1 = x2 + y2 + z2 - a*k*k;
    const float term2 = (z-k)*(z-k) - 2*x2;
    const float term3 = (z+k)*(z+k) - 2*y2;

    return term1*term1 - b*term2*term3;
#elif SURFACE == 2 //heart
    //http://mathworld.wolfram.com/HeartSurface.html
    const float z3 = z2*z;
    const float term1 = x2 + (9.f/4)*y2 + z2 - 1;
    return term1*term1*term1 - x2*z3 -(9.f/80)*y2*z3;
#endif
}

//bool ImplicitSurface::Intersect(const Ray &ray, Intersection *isect) const {
//    //r_loc is in local space
//    Ray r_loc = ray.GetTransformedCopy(transform.invT());
//    int MAX_MARCHING_STEPS = 100000;
//    float END = 10000;
//    float depth = 0.01;
//    for(int i = 0; i < MAX_MARCHING_STEPS; i++){
//        //DEBUG: signedDistFunc returns large numbers ex: 79392676
//        float dist = signedDistFunc(r_loc.origin + depth*r_loc.direction);

//        if(dist < 0.005) {//inside
//            InitializeIntersection(isect, depth, r_loc.origin + depth*r_loc.direction);
//            true;
//        }

//        depth += dist - 0.01;

//        if(depth >= END) {
//            std::cout << "\nQuitting with depth: " << depth;
//            return false;
//        }
//    }
//    return false;
//}

bool signSwitched(const float startingsign, const float dist) {
    const float thresh = 0.005;
    if(startingsign == 1){
        if(dist < thresh) { return true; }
    } else {//started inside
        if(dist > -thresh) { return true; }
    }
    return false;
}

bool ImplicitSurface::Intersect(const Ray &ray, Intersection *isect) const {
    //r_loc is in local space
    Ray r_loc = ray.GetTransformedCopy(transform.invT());
    const float farplane = 100;
    float step = 0.1;
    const int MAX_MARCHING_STEPS = farplane * (1.f/step);
    const float babystep = 0.001;
    const float startingsign = glm::sign(signedDistFunc(r_loc.origin));
    float t = 0;
    bool finegrainmode = false;
    const float bias = 0.005 * startingsign;
    for(int i = 0; i < MAX_MARCHING_STEPS; i++){
        t += step;
        float dist = signedDistFunc(r_loc.origin + t*r_loc.direction);

//        if(dist < 0.005) {//inside
        if(signSwitched(startingsign, dist)) {
            if(!finegrainmode) {
                t -= step;
                step = babystep;
                finegrainmode = true;
                i -= (step/babystep);//for cases where intersection is almost at far plane, want to ajust i so we get there with the baby steps
            } else {
                t -= bias;
                InitializeIntersection(isect, t, r_loc.origin + t*r_loc.direction);
                return true;
            }
        }
    }
    return false;
}


void ImplicitSurface::ComputeTBN(const Point3f& P, Normal3f* nor, Vector3f* tan, Vector3f* bit) const {
    //P is intersection in local space
    const float delta = 0.005;
    Point3f localnormal = glm::normalize(Vector3f(
                          signedDistFunc(Point3f(P.x + delta, P.y, P.z)) - signedDistFunc(Point3f(P.x - delta, P.y, P.z))
                         ,signedDistFunc(Point3f(P.x, P.y + delta, P.z)) - signedDistFunc(Point3f(P.x, P.y - delta, P.z))
                         ,signedDistFunc(Point3f(P.x, P.y, P.z + delta)) - signedDistFunc(Point3f(P.x, P.y, P.z - delta))
                          ));
    *nor = glm::normalize(transform.invTransT() * localnormal);
    *tan = glm::normalize(transform.T3() * glm::cross(Vector3f(0,1,0), localnormal));
    *bit = glm::normalize(glm::cross(*nor, *tan));
}

Bounds3f ImplicitSurface::WorldBound() const {
    float scale = 3.f;//dont know size of implicit surface so estimate with cube
    Bounds3f modelbounds(scale*Point3f(-1.f, -1.f, -1.f), scale*Point3f(1.f, 1.f, 1.f));
    return modelbounds.Apply(this->transform);
}
