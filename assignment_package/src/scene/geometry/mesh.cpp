#include <scene/geometry/mesh.h>
#include <la.h>
#include <tinyobj/tiny_obj_loader.h>
#include <iostream>

Bounds3f Triangle::WorldBound() const
{
    //TODO
    //mins
    const float minx = std::min( std::min(points[0].x, points[1].x), points[2].x);
    const float miny = std::min( std::min(points[0].y, points[1].y), points[2].y);
    const float minz = std::min( std::min(points[0].z, points[1].z), points[2].z);
    //maxs
    const float maxx = std::max( std::max(points[0].x, points[1].x), points[2].x);
    const float maxy = std::max( std::max(points[0].y, points[1].y), points[2].y);
    const float maxz = std::max( std::max(points[0].z, points[1].z), points[2].z);

    Bounds3f modelbounds(Point3f(minx, miny, minz), Point3f(maxx, maxy, maxz));
    return modelbounds.Apply(this->transform);
}

float Triangle::Area() const
{
    return glm::length(glm::cross(points[0] - points[1], points[2] - points[1])) * 0.5f;
}

void Triangle::getSurfaceEmissionRay(const Point2f &xi, const Point2f& xk, Ray& ray ,float& pdf) const {
    pdf = 1.f / Area();
    // section 4.2 of http://www.cs.princeton.edu/~funk/tog02.pdf
    const float sqrtxix = sqrtf(xi.x);
    Point3f surfpoint = (1.f-sqrtxix)*points[0] + (sqrtxix*(1.f-xi.y))*points[1] + xi.y*sqrtxix*points[2];
    surfpoint +=  (0.005f * GetNormal(surfpoint));
    const glm::vec4 p                 = transform.T() * glm::vec4(surfpoint.x, surfpoint.y, surfpoint.z,1.f);
    ray.origin = Vector3f(p.x, p.y, p.z);
    const Vector3f dir = WarpFunctions::squareToHemisphereUniform(xk);
    ray.direction      = glm::normalize(transform.invTransT() * dir);
}
Triangle::Triangle(const glm::vec3 &p1, const glm::vec3 &p2, const glm::vec3 &p3):
    Triangle(p1, p2, p3, glm::vec3(1), glm::vec3(1), glm::vec3(1), glm::vec2(0), glm::vec2(0), glm::vec2(0))
{
    for(int i = 0; i < 3; i++)
    {
        normals[i] = planeNormal;
    }
}


Triangle::Triangle(const glm::vec3 &p1, const glm::vec3 &p2, const glm::vec3 &p3, const glm::vec3 &n1, const glm::vec3 &n2, const glm::vec3 &n3):
    Triangle(p1, p2, p3, n1, n2, n3, glm::vec2(0), glm::vec2(0), glm::vec2(0))
{}


Triangle::Triangle(const glm::vec3 &p1, const glm::vec3 &p2, const glm::vec3 &p3, const glm::vec3 &n1, const glm::vec3 &n2, const glm::vec3 &n3, const glm::vec2 &t1, const glm::vec2 &t2, const glm::vec2 &t3)
{
    planeNormal = glm::normalize(glm::cross(p2 - p1, p3 - p2));
    points[0] = p1;
    points[1] = p2;
    points[2] = p3;
    normals[0] = n1;
    normals[1] = n2;
    normals[2] = n3;
    uvs[0] = t1;
    uvs[1] = t2;
    uvs[2] = t3;
}

float TriArea(const glm::vec3 &p1, const glm::vec3 &p2, const glm::vec3 &p3)
{
    return glm::length(glm::cross(p1 - p2, p3 - p2)) * 0.5f;
}

//Returns the interpolation of the triangle's three normals based on the point inside the triangle that is given.
Normal3f Triangle::GetNormal(const Point3f &P) const
{
    float A = TriArea(points[0], points[1], points[2]);
    float A0 = TriArea(points[1], points[2], P);
    float A1 = TriArea(points[0], points[2], P);
    float A2 = TriArea(points[0], points[1], P);
    return glm::normalize(normals[0] * A0/A + normals[1] * A1/A + normals[2] * A2/A);
}


//The ray in this function is not transformed because it was *already* transformed in Mesh::GetIntersection
bool Triangle::Intersect(const Ray& r, Intersection* isect) const
{
    //1. Ray-plane intersection
    float t =  glm::dot(planeNormal, (points[0] - r.origin)) / glm::dot(planeNormal, r.direction);
    if(t < 0) return false;

    glm::vec3 P = r.origin + t * r.direction;
    //2. Barycentric test
    float S = 0.5f * glm::length(glm::cross(points[0] - points[1], points[0] - points[2]));
    float s1 = 0.5f * glm::length(glm::cross(P - points[1], P - points[2]))/S;
    float s2 = 0.5f * glm::length(glm::cross(P - points[2], P - points[0]))/S;
    float s3 = 0.5f * glm::length(glm::cross(P - points[0], P - points[1]))/S;
    float sum = s1 + s2 + s3;

    if(s1 >= 0 && s1 <= 1 && s2 >= 0 && s2 <= 1 && s3 >= 0 && s3 <= 1 && fequal(sum, 1.0f)){
        isect->t = t;
        InitializeIntersection(isect, t, Point3f(P));
        return true;
    }
    return false;
}

void Triangle::InitializeIntersection(Intersection *isect, float t, Point3f pLocal) const
{
    isect->point = pLocal;
    isect->uv = GetUVCoordinates(pLocal);
    ComputeTriangleTBN(pLocal, &(isect->normalGeometric), &(isect->tangent), &(isect->bitangent), isect->uv);
    isect->t = t;
}

void Triangle::ComputeTBN(const Point3f &P, Normal3f *nor, Vector3f *tan, Vector3f *bit) const
{
    //Triangle uses ComputeTriangleTBN instead of this function.
}

void Triangle::ComputeTriangleTBN(const Point3f &P, Normal3f *nor, Vector3f *tan, Vector3f *bit, const Point2f &uv) const
{
    *nor = GetNormal(P);
    //TODO: Compute tangent and bitangent based on UV coordinates.
    Point3f p0 = points[0];
    Point3f p1 = points[1];
    Point3f p2 = points[2];

    Point2f uv0 = uvs[0];
    Point2f uv1 = uvs[1];
    Point2f uv2 = uvs[2];

    Point3f deltaPos1 = p1-p0;
    Point3f deltaPos2 = p2-p0;

    Point2f deltaUV1 = uv1-uv0;
    Point2f deltaUV2 = uv2-uv0;

    float r = 1.0f / (deltaUV1.x * deltaUV2.y - deltaUV1.y * deltaUV2.x);

    if ((deltaUV1.x * deltaUV2.y - deltaUV1.y * deltaUV2.x) == 0.f) {
        *tan = glm::normalize(p1 - p0);
        *bit = glm::normalize(glm::cross(*nor, *tan));
    } else {
        *tan = glm::normalize((deltaPos1 * deltaUV2.y - deltaPos2 * deltaUV1.y)*r);
        *bit = glm::normalize((deltaPos2 * deltaUV1.x - deltaPos1 * deltaUV2.x)*r);
    }
}


//void Triangle::ComputeTriangleTBN(const Point3f &P, Normal3f *nor, Vector3f *tan, Vector3f *bit, const Point2f &uv) const
//{
//    //TODO: Compute tangent and bitangent based on UV coordinates.
//    //This is Shading Normal, not Geometric Normal
//    *nor = GetNormal(P);

//    //TODO: Compute tangent and bitangent based on UV coordinates.
//    Vector3f e1 = points[1] - points[0];
//    Vector3f e2 = points[2] - points[0];

//    //This is the Geometric Normal, assuming the points go ccw
//    Normal3f gnor = glm::normalize( glm::cross(e1, e2) );

//    Vector3f axisangle = glm::cross(gnor, *nor);

//    //projected lengths of the triangle edges along the UV axes
//    Vector2f e1_dUVs = uvs[1] - uvs[0];
//    Vector2f e2_dUVs = uvs[2] - uvs[0];

//    //if the uv's stored at the vertices are skewed to get the texture
//    //to warp correctly across the surface then our T and B may be skewed as well
//    Vector3f tangent = (e2_dUVs.y * e1 - e1_dUVs.y * e2) /
//                        (e2_dUVs.y * e1_dUVs.x - e1_dUVs.y * e2_dUVs.x);
//    Vector3f bitan = (e2 - e2_dUVs.x * tangent) / e2_dUVs.y;


////    if(glm::length(axisangle) > 0.01) {
////        //theta, sintheta, costheta
////        glm::vec4 quat;
////    }

//    *tan = glm::normalize(tangent);
//    *bit = glm::normalize(bitan);
//}


Intersection Triangle::Sample(const Point2f &xi, Float *pdf) const
{
    //TODO for extra credit
    return Intersection();
}


Point2f Triangle::GetUVCoordinates(const Point3f &point) const
{
    float A = TriArea(points[0], points[1], points[2]);
    float A0 = TriArea(points[1], points[2], point);
    float A1 = TriArea(points[0], points[2], point);
    float A2 = TriArea(points[0], points[1], point);
    return uvs[0] * A0/A + uvs[1] * A1/A + uvs[2] * A2/A;
}

void Mesh::LoadOBJ(const QStringRef &filename, const QStringRef &local_path, const Transform &transform)
{
    QString filepath = local_path.toString(); filepath.append(filename);
    std::vector<tinyobj::shape_t> shapes; std::vector<tinyobj::material_t> materials;
    std::string errors = tinyobj::LoadObj(shapes, materials, filepath.toStdString().c_str());
    std::cout << errors << std::endl;
    if(errors.size() == 0)
    {
        //Read the information from the vector of shape_ts
        for(unsigned int i = 0; i < shapes.size(); i++)
        {
            std::vector<float> &positions = shapes[i].mesh.positions;
            std::vector<float> &normals = shapes[i].mesh.normals;
            std::vector<float> &uvs = shapes[i].mesh.texcoords;
            std::vector<unsigned int> &indices = shapes[i].mesh.indices;
            for(unsigned int j = 0; j < indices.size(); j += 3)
            {
                glm::vec3 p1 = glm::vec3(transform.T() * glm::vec4(positions[indices[j]*3], positions[indices[j]*3+1], positions[indices[j]*3+2], 1));
                glm::vec3 p2 = glm::vec3(transform.T() * glm::vec4(positions[indices[j+1]*3], positions[indices[j+1]*3+1], positions[indices[j+1]*3+2], 1));
                glm::vec3 p3 = glm::vec3(transform.T() * glm::vec4(positions[indices[j+2]*3], positions[indices[j+2]*3+1], positions[indices[j+2]*3+2], 1));

                auto t = std::make_shared<Triangle>(p1, p2, p3);
                if(normals.size() > 0)
                {
                    glm::vec3 n1 = transform.invTransT() * glm::vec3(normals[indices[j]*3], normals[indices[j]*3+1], normals[indices[j]*3+2]);
                    glm::vec3 n2 = transform.invTransT() * glm::vec3(normals[indices[j+1]*3], normals[indices[j+1]*3+1], normals[indices[j+1]*3+2]);
                    glm::vec3 n3 = transform.invTransT() * glm::vec3(normals[indices[j+2]*3], normals[indices[j+2]*3+1], normals[indices[j+2]*3+2]);
                    t->normals[0] = n1;
                    t->normals[1] = n2;
                    t->normals[2] = n3;
                }
                if(uvs.size() > 0)
                {
                    glm::vec2 t1(uvs[indices[j]*2], uvs[indices[j]*2+1]);
                    glm::vec2 t2(uvs[indices[j+1]*2], uvs[indices[j+1]*2+1]);
                    glm::vec2 t3(uvs[indices[j+2]*2], uvs[indices[j+2]*2+1]);
                    t->uvs[0] = t1;
                    t->uvs[1] = t2;
                    t->uvs[2] = t3;
                }
                this->faces.append(t);
            }
        }
        std::cout << "" << std::endl;
        //TODO: .mtl file loading
    }
    else
    {
        //An error loading the OBJ occurred!
        std::cout << errors << std::endl;
    }
}
