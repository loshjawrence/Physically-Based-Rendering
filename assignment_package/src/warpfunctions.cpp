#define _USE_MATH_DEFINES
#include "warpfunctions.h"
#include <math.h>
#include <globals.h>

glm::vec3 WarpFunctions::squareToDiskUniform(const glm::vec2 &sample)
{
    float r = std::sqrt(sample.x);
    float theta = 2.f * Pi * sample.y;
    return glm::vec3(r * std::cos(theta), r * std::sin(theta), 0.f);
}

glm::vec3 WarpFunctions::squareToDiskConcentric(const glm::vec2 &sample)
{
    //map uniform random numbers -1,1
    glm::vec2 offset = 2.f * sample - glm::vec2(1.f,1.f);

    //handle degeneracy at the origin
    if(offset.x == 0.f && offset.y == 0.f) {
        return glm::vec3(0.f,0.f,0.f);
    }

    float theta, r;
    if(std::fabs(offset.x) > std::fabs(offset.y)) {
        r = offset.x;
        theta = (Pi / 4.f) * (offset.y/offset.x);
    } else {
        r = offset.y;
        theta = (Pi / 2.f) - (Pi / 4.f) * (offset.x/offset.y);
    }

    return r * glm::vec3(std::cos(theta), std::sin(theta), 0.f);
}

float WarpFunctions::squareToDiskPDF(const glm::vec3 &sample)
{
    return InvPi;
}

glm::vec3 WarpFunctions::squareToSphereUniform(const glm::vec2 &sample)
{
    //maps x to value between 1 and -1
    float z = 1.f - 2.f * sample.x;
    float r = std::sqrt(std::max(0.f, 1.f - z * z));
    float phi = 2.f * Pi * sample.y;
    return glm::vec3(r * std::cos(phi), r * std::sin(phi), z);
}

float WarpFunctions::squareToSphereUniformPDF(const glm::vec3 &sample)
{
    return Inv4Pi;
}

float WarpFunctions::squareToSphereCosinePDF(const glm::vec3 &sample)
{
    float cosTheta = AbsDot(sample,glm::vec3(0.f,0.f,1.f));
    return cosTheta * Inv2Pi;
}

glm::vec3 WarpFunctions::squareToSphereCapUniform(const glm::vec2 &sample, float thetaMin)
{
    //costhetamin is our min z, max z is 1
    float thetaMax = glm::radians(180.f - thetaMin);
    float zmin = std::cos(thetaMax);
    float zmax = 1.f;
    //
    float z = zmax - (zmax - zmin) * sample.x;
    float r = std::sqrt(std::max(0.f, 1.f - z * z));
    float phi = 2.f * Pi * sample.y;
    return glm::vec3(r * std::cos(phi), r * std::sin(phi), z);
}

float WarpFunctions::squareToSphereCapUniformPDF(const glm::vec3 &sample, float thetaMin)
{
    //integrating over full Surface of sphere 1/A = 1/4PIr*r = 1/4PI
    //so subtract off surface area that we are not including theta min
    thetaMin = glm::radians(thetaMin);
    float theta_integral = 1.f - std::cos(Pi - thetaMin);
    float phi_integral = 2.f * Pi;
    return 1.f / (phi_integral * theta_integral);

}

glm::vec3 WarpFunctions::squareToHemisphereUniform(const glm::vec2 &sample)
{
    float z = sample.x;
    float r = std::sqrt(std::max(0.f, 1.f - z * z));
    float phi = 2 * Pi * sample.y;
    return glm::vec3(r * std::cos(phi), r * std::sin(phi), z);
}

float WarpFunctions::squareToHemisphereUniformPDF(const glm::vec3 &sample)
{
    return Inv2Pi;
}

glm::vec3 WarpFunctions::squareToHemisphereCosine(const glm::vec2 &sample)
{
    glm::vec3 d = WarpFunctions::squareToDiskConcentric(sample);
    float z = std::sqrt(std::max(0.f, 1.f - d.x * d.x - d.y * d.y));
    return glm::vec3(d.x, d.y, z);
}

float WarpFunctions::squareToHemisphereCosinePDF(const glm::vec3 &sample)
{
    float cosTheta = glm::dot(sample,glm::vec3(0.f,0.f,1.f));
    return cosTheta * InvPi;
}
