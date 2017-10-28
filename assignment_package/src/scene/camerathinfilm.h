//#pragma once

//#include <scene/camera.h>
//#include <pcg32.h>
//#include <warpfunctions.h>

//class CameraThinFilm : public Camera
//{
//public:
//    CameraThinFilm();
//    CameraThinFilm(unsigned int w, unsigned int h);
//    CameraThinFilm(unsigned int w, unsigned int h, const Vector3f &e, const Vector3f &r, const Vector3f &worldUp, Float lensr, Float focald);
//    CameraThinFilm(const CameraThinFilm &c);


//    Float lensRadius,focalDistance;
//    pcg32 rng;

//    float fovy;
//    unsigned int width, height;  // Screen dimensions
//    float near_clip;  // Near clip plane distance
//    float far_clip;  // Far clip plane distance

//    //Computed attributes
//    float aspect;

//    Point3f  eye,      //The position of the camera in world space
//             ref;      //The point in world space towards which the camera is pointing
//    Vector3f look,     //The normalized vector from eye to ref. Is also known as the camera's "forward" vector.
//             up,       //The normalized vector pointing upwards IN CAMERA SPACE. This vector is perpendicular to LOOK and RIGHT.
//             right,    //The normalized vector pointing rightwards IN CAMERA SPACE. It is perpendicular to UP and LOOK.
//             world_up, //The normalized vector pointing upwards IN WORLD SPACE. This is primarily used for computing the camera's initial UP vector.
//             V,        //Represents the vertical component of the plane of the viewing frustum that passes through the camera's reference point. Used in Camera::Raycast.
//             H;        //Represents the horizontal component of the plane of the viewing frustum that passes through the camera's reference point. Used in Camera::Raycast.

//    void CopyAttributes(const CameraThinFilm &c);

//    Ray Raycast(const Point2f &pt) ;         //Creates a ray in 3D space given a 2D point on the screen, in screen coordinates.
//    Ray Raycast(float x, float y) ;            //Same as above, but takes two floats rather than a vec2.
//    Ray RaycastNDC(float ndc_x, float ndc_y); //Creates a ray in 3D space given a 2D point in normalized device coordinates.
//};

