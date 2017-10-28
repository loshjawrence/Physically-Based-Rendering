//#include "camerathinfilm.h"
//#include <iostream>

//CameraThinFilm::CameraThinFilm():
//    Camera(),rng()
//{
//}

//CameraThinFilm::CameraThinFilm(unsigned int w, unsigned int h):
//    Camera(w, h),rng()
//{}

//CameraThinFilm::CameraThinFilm(unsigned int w, unsigned int h, const Vector3f &e, const Vector3f &r, const Vector3f &worldUp,Float lensr, Float focald):
//    Camera(w,h,e,r,worldUp),lensRadius(lensr), focalDistance(focald), rng()
//{
//    RecomputeAttributes();
//}

//CameraThinFilm::CameraThinFilm(const CameraThinFilm &c):
//    fovy(c.fovy),
//    width(c.width),
//    height(c.height),
//    near_clip(c.near_clip),
//    far_clip(c.far_clip),
//    aspect(c.aspect),
//    eye(c.eye),
//    ref(c.ref),
//    look(c.look),
//    up(c.up),
//    right(c.right),
//    world_up(c.world_up),
//    V(c.V),
//    H(c.H),
//    lensRadius(c.lensRadius),
//    focalDistance(c.focalDistance)
//{}

//void CameraThinFilm::CopyAttributes(const CameraThinFilm &c)
//{
//    fovy = c.fovy;
//    near_clip = c.near_clip;
//    far_clip = c.far_clip;
//    eye = c.eye;
//    ref = c.ref;
//    look = c.look;
//    up = c.up;
//    right = c.right;
//    width = c.width;
//    height = c.height;
//    aspect = c.aspect;
//    V = c.V;
//    H = c.H;
//    lensRadius = c.lensRadius;
//    focalDistance = c.focalDistance;
//}

//Ray CameraThinFilm::Raycast(const Point2f &pt)
//{
//    return Raycast(pt.x, pt.y);
//}

//Ray CameraThinFilm::Raycast(float x, float y)
//{
//    float ndc_x = (2.f*x/width - 1);
//    float ndc_y = (1 - 2.f*y/height);
//    return RaycastNDC(ndc_x, ndc_y);
//}

//Ray CameraThinFilm::RaycastNDC(float ndc_x, float ndc_y)
//{
//    glm::vec3 P = ref + ndc_x*H + ndc_y*V;
//    Ray result(eye, glm::normalize(P - eye));


////    std::cout << "\n\nRaycastNDC lensRadius: " << lensRadius;

//    //thin film camera pbrtv3 page 375
//    if(lensRadius > 0) {
//        //world to camera, aka inverse of camera to world aka view matrix
//        glm::mat4 worldtocam = glm::lookAt(eye, ref, up);
//        glm::vec4 corigin  = worldtocam * glm::vec4(result.origin, 1.0f);
//        glm::vec4 cdirection = worldtocam * glm::vec4(result.direction, 0.0f);

//        result = Ray(glm::vec3(corigin), glm::vec3(cdirection));

//        Point3f pLens = lensRadius *
//                WarpFunctions::squareToDiskConcentric(Point2f(rng.nextFloat(), rng.nextFloat()));
////        std::cout << "\n\nlensRadius: " << lensRadius << "\npLens: " << pLens.x << " " << pLens.y << " " << pLens.z;
//        Float ft = glm::abs(focalDistance / result.direction.z);
//        Point3f pFocus = result.origin + ft*result.direction;
//        result.origin = pLens;
//        result.direction = glm::normalize(pFocus - result.origin);

//        //camera to world
//        glm::mat4 camtoworld = glm::inverse(worldtocam);
//        glm::vec4 worigin = camtoworld * glm::vec4(result.origin, 1.0f);
//        glm::vec4 wdirection = camtoworld * glm::vec4(result.direction, 0.0f);

//        result = Ray(glm::vec3(worigin), glm::vec3(wdirection));
//    }
//    return result;
//}

