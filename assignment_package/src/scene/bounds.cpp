#include "bounds.h"

bool Bounds3f::Intersect(const Ray &r, float* t) const {
    //TODO
    float t_n = -FLT_MAX;
    float t_f = FLT_MAX;
    const Point2f minmax_xyz[3] = {Point2f(min.x, max.x), Point2f(min.y, max.y), Point2f(min.z, max.z)};
    for(int i = 0; i < 3; i++){
        //Ray parallel to slab check
        const float minbound = minmax_xyz[i][0];
        const float maxbound = minmax_xyz[i][1];
        if(r.direction[i] == 0){
            if(r.origin[i] < minbound || r.origin[i] > maxbound){
                return false;
            }
        }
        //If not parallel, do slab intersect check
        float t0 = (minbound - r.origin[i])/r.direction[i];
        float t1 = (maxbound - r.origin[i])/r.direction[i];
        if(t0 > t1) {
            float temp = t1;
            t1 = t0;
            t0 = temp;
        }
        if(t0 > t_n) {
            t_n = t0;
        }
        if(t1 < t_f) {
            t_f = t1;
        }
    }
    if(t_n < t_f) {
        if(r.origin.x >= minmax_xyz[0][0] && r.origin.y >= minmax_xyz[1][0] && r.origin.z >= minmax_xyz[2][0]
        && r.origin.x <= minmax_xyz[0][1] && r.origin.y <= minmax_xyz[1][1] && r.origin.z <= minmax_xyz[2][1]){
            *t = t_n;
            return true;
        } else {
            float temp_t = t_n > 0 ? t_n : t_f;
            if(temp_t < 0) {
                return false;
            }
            *t = temp_t;
        }
        return true;
    } else {//If t_near was greater than t_far, we did not hit the cube
        return false;
    }
}

Bounds3f Bounds3f::Apply(const Transform &tr) {
    //top ring, ccw starting from front right
    const glm::vec4 p1 = tr.T() * glm::vec4(max.x, max.y, max.z, 1);
    const glm::vec4 p2 = tr.T() * glm::vec4(min.x, max.y, max.z, 1);
    const glm::vec4 p3 = tr.T() * glm::vec4(min.x, max.y, min.z, 1);
    const glm::vec4 p4 = tr.T() * glm::vec4(max.x, max.y, min.z, 1);
    const float mintopx = std::min( std::min( std::min(p1.x,p2.x), p3.x), p4.x);
    const float mintopy = std::min( std::min( std::min(p1.y,p2.y), p3.y), p4.y);
    const float mintopz = std::min( std::min( std::min(p1.z,p2.z), p3.z), p4.z);
    const float maxtopx = std::max( std::max( std::max(p1.x,p2.x), p3.x), p4.x);
    const float maxtopy = std::max( std::max( std::max(p1.y,p2.y), p3.y), p4.y);
    const float maxtopz = std::max( std::max( std::max(p1.z,p2.z), p3.z), p4.z);

    //bottom ring, cw starting from back left
    const glm::vec4 p5 = tr.T() * glm::vec4(min.x, min.y, min.z, 1);
    const glm::vec4 p6 = tr.T() * glm::vec4(max.x, min.y, min.z, 1);
    const glm::vec4 p7 = tr.T() * glm::vec4(max.x, min.y, max.z, 1);
    const glm::vec4 p8 = tr.T() * glm::vec4(min.x, min.y, max.z, 1);
    const float minbotx = std::min( std::min( std::min(p5.x,p6.x), p7.x), p8.x);
    const float minboty = std::min( std::min( std::min(p5.y,p6.y), p7.y), p8.y);
    const float minbotz = std::min( std::min( std::min(p5.z,p6.z), p7.z), p8.z);
    const float maxbotx = std::max( std::max( std::max(p5.x,p6.x), p7.x), p8.x);
    const float maxboty = std::max( std::max( std::max(p5.y,p6.y), p7.y), p8.y);
    const float maxbotz = std::max( std::max( std::max(p5.z,p6.z), p7.z), p8.z);

    min = Point3f(std::min(mintopx, minbotx),
                  std::min(mintopy, minboty),
                  std::min(mintopz, minbotz));

    max = Point3f(std::max(maxtopx, maxbotx),
                  std::max(maxtopy, maxboty),
                  std::max(maxtopz, maxbotz));

    return Bounds3f(min,max);
}

float Bounds3f::SurfaceArea() const {
    //TODO
    Vector3f d = Diagonal();
    return 2.f*(d.x*d.y + d.x*d.z + d.y*d.z);
}

Bounds3f Union(const Bounds3f& b1, const Bounds3f& b2) {
    return Bounds3f(Point3f(std::min(b1.min.x, b2.min.x),
                            std::min(b1.min.y, b2.min.y),
                            std::min(b1.min.z, b2.min.z)),
                    Point3f(std::max(b1.max.x, b2.max.x),
                            std::max(b1.max.y, b2.max.y),
                            std::max(b1.max.z, b2.max.z)));
}

Bounds3f Union(const Bounds3f& b1, const Point3f& p) {
    return Bounds3f(Point3f(std::min(b1.min.x, p.x),
                            std::min(b1.min.y, p.y),
                            std::min(b1.min.z, p.z)),
                    Point3f(std::max(b1.max.x, p.x),
                            std::max(b1.max.y, p.y),
                            std::max(b1.max.z, p.z)));
}

Bounds3f Union(const Bounds3f& b1, const glm::vec4& p) {
    return Union(b1, Point3f(p));
}
