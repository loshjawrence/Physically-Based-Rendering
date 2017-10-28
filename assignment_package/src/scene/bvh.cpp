#include "bvh.h"
#define BINARY_TREE 0
// Feel free to ignore these structs entirely!
// They are here if you want to implement any of PBRT's
// methods for BVH construction.

struct BVHPrimitiveInfo {
    BVHPrimitiveInfo() {}
    BVHPrimitiveInfo(size_t primitiveNumber, const Bounds3f &bounds)
        : primitiveNumber(primitiveNumber),
          bounds(bounds),
          centroid(.5f * bounds.min + .5f * bounds.max) {}
    int primitiveNumber;
    Bounds3f bounds;
    Point3f centroid;
};

struct BVHBuildNode {
    // BVHBuildNode Public Methods
    void InitLeaf(int first, int n, const Bounds3f &b) {
        firstPrimOffset = first;
        nPrimitives = n;
        bounds = b;
        children[0] = children[1] = nullptr;
    }
    void InitInterior(int axis, BVHBuildNode *c0, BVHBuildNode *c1) {
        children[0] = c0;
        children[1] = c1;
        bounds = Union(c0->bounds, c1->bounds);
        splitAxis = axis;
        nPrimitives = 0;
    }
    Bounds3f bounds;
    BVHBuildNode *children[2];
    int splitAxis, firstPrimOffset, nPrimitives;
};

struct MortonPrimitive {
    int primitiveIndex;
    unsigned int mortonCode;
};

struct LBVHTreelet {
    int startIndex, nPrimitives;
    BVHBuildNode *buildNodes;
};

struct LinearBVHNode {
    Bounds3f bounds;
    union {
        int primitivesOffset;   // leaf
        int secondChildOffset;  // interior
    };
    unsigned short nPrimitives;  // 0 -> interior node, 16 bytes
    unsigned char axis;          // interior node: xyz, 8 bytes
    unsigned char pad[1];        // ensure 32 byte total size
};


#if BINARY_TREE == 1
struct BVHNode {
    // BVHNode Public Methods
    BVHNode(){}
    BVHNode(Bounds3f& thebounds, int numprims)
        : bounds(thebounds), nPrimitives(numprims),children{nullptr, nullptr},myprimitives(0) {

    }
    Bounds3f bounds;
    BVHNode *children[2];
    int splitAxis, nPrimitives;
    std::vector<std::shared_ptr<Primitive>> myprimitives;
//    std::vector<Primitive*> myprimitives;
};

struct PrimitiveData {
    PrimitiveData() {}
    PrimitiveData(Bounds3f& bounds, std::shared_ptr<Primitive> prim)
        : bounds(bounds), centroid(.5f*(bounds.min + bounds.max)), myprimitive(prim) {}
//    PrimitiveData(Bounds3f& bounds, Primitive* prim)
//        : bounds(bounds), centroid(.5f*(bounds.min + bounds.max)), myprimitive(prim) {}
    Bounds3f bounds;
    Point3f centroid;
    std::shared_ptr<Primitive> myprimitive;
//    Primitive* myprimitive;
};
#endif

BVHAccel::~BVHAccel()
{
    delete nodes;
}

#if BINARY_TREE == 1
// Constructs an array of BVHPrimitiveInfos, recursively builds a node-based BVH
// from the information, then optimizes the memory of the BVH
BVHAccel::BVHAccel(const std::vector<std::shared_ptr<Primitive> > &p, int maxPrimsInNode)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), primitives(p)
{
    //TODO
    const int total = primitives.size();
    if(total == 0) {
        return;
    }

    //construct primitiveData
    std::vector<PrimitiveData> primitiveData;
    primitiveData.reserve(primitives.size());
    primitiveData.resize(primitives.size());
    for(int i = 0; i < total; ++i) {
        std::shared_ptr<Primitive> myprim = primitives.at(i);
//        Primitive* myprim = primitives[i].get();
        Bounds3f mybounds = myprim->WorldBound();
        primitiveData[i] = PrimitiveData(mybounds,myprim);
    }

    int totalNodes = 0;
    nodes = recurse(primitiveData, 0, total, &totalNodes);
    std::cout << "\nNum primitives: " << total << "\nNum Nodes: " << totalNodes << "\n2n-1: " << 2*total-1;
}

BVHNode* BVHAccel::recurse( std::vector<PrimitiveData> &primitiveData, int start, int end, int* totalNodes) {
    (*totalNodes)++;
//    std::cout << "\nstart:" << start << " end:" << end;
    if(start == end-1) {
        PrimitiveData data = primitiveData[start];
        BVHNode* newnode = new BVHNode(data.bounds,1);
        newnode->myprimitives.push_back(data.myprimitive);
        return newnode;
    }

    //CREATE AABB
    Bounds3f nodebounds(Point3f(FLT_MAX, FLT_MAX, FLT_MAX), Point3f(-FLT_MAX, -FLT_MAX, -FLT_MAX));
    for(int i = start; i < end; ++i) {
        PrimitiveData prim = primitiveData[i];
        nodebounds.min.x = std::min( prim.bounds.min.x, nodebounds.min.x);
        nodebounds.min.y = std::min( prim.bounds.min.y, nodebounds.min.y);
        nodebounds.min.z = std::min( prim.bounds.min.z, nodebounds.min.z);
        nodebounds.max.x = std::max( prim.bounds.max.x, nodebounds.max.x);
        nodebounds.max.y = std::max( prim.bounds.max.y, nodebounds.max.y);
        nodebounds.max.z = std::max( prim.bounds.max.z, nodebounds.max.z);
    }


    //SORT ALONG LARGEST SPAN
    // 0 == x
    // 1 == y
    // 2 == z
    const int dimtosplit = nodebounds.MaximumExtent();
    const Vector3f d = nodebounds.Diagonal();

    // 0 == xy
    // 1 == xz
    // 2 == yz
    int areaplane = 0;
    if(dimtosplit == 0) {
        if(d.z >= d.y) { areaplane = 1; }
    } else if(dimtosplit == 1) {
        if(d.z >= d.x) { areaplane = 2; }
    } else {
        areaplane = 1;
        if(d.y >= d.z) { areaplane = 2; }
    }

    std::sort(&primitiveData[start],&primitiveData[end-1]+1,
            [dimtosplit](const PrimitiveData& p1, const PrimitiveData& p2) {
        return p1.bounds.max[dimtosplit] <= p2.bounds.max[dimtosplit];
    });

    Bounds3f leftbounds = primitiveData[start].bounds;
    Bounds3f rightbounds;
    float leftarea  = 0;
    float rightarea = 0;
    int mid = start + 1;
    while (mid < end) {
        leftbounds = Union(leftbounds,primitiveData[mid-1].bounds);
        rightbounds = primitiveData[mid].bounds;
        //finish finding the right bounds
        for(int i = mid+1; i < end; ++i){
            rightbounds = Union(rightbounds,primitiveData[i].bounds);
        }

        //FIND AREAS
        Vector3f dleft  = leftbounds.Diagonal();
        Vector3f dright = rightbounds.Diagonal();
        if(areaplane == 0) {
            leftarea  =  dleft.x *  dleft.y;
            rightarea = dright.x * dright.y;
        } else if (areaplane == 1) {
            leftarea  =  dleft.x *  dleft.z;
            rightarea = dright.x * dright.z;
        } else {
            leftarea  =  dleft.y *  dleft.z;
            rightarea = dright.y * dright.z;
        }

        //check if 'sign' switched
        if(leftarea > rightarea) {
            break;
        }
        mid++;
    }

    //case where rightmost bounding box was more area than the rest so the sign never switched
    //set mid to be index of last object
    if(mid == end) { mid = end-1; }

    //DETERMINE COST
    int numleftprims  = mid - start;
    int numrightprims = end - mid;
    float totalprims  = end - start;
    float encompassingbound = 0;
    if(areaplane == 0) {
        encompassingbound  =  d.x *  d.y;
    } else if (areaplane == 1) {
        encompassingbound  =  d.x *  d.z;
    } else {
        encompassingbound  =  d.y *  d.z;
    }

    float cost = ( leftarea * numleftprims + rightarea * numrightprims ) / encompassingbound;
    if( totalprims <= maxPrimsInNode || cost >= totalprims) {
        //package up remaining prims in a BVHNode and return it
        BVHNode* newnode = new BVHNode(nodebounds,totalprims);
//        newnode->myprimitives.reserve(totalprims);
//        newnode->myprimitives.resize(totalprims);
        for(int i = start; i < end; ++i){
            PrimitiveData myprimdata = primitiveData[i];
            newnode->myprimitives.push_back(myprimdata.myprimitive);
//            newnode->myprimitives[i] = myprimdata.myprimitive;
        }
        return newnode;
    } else {
        BVHNode* newnode = new BVHNode(nodebounds,totalprims);
        newnode->children[0] = recurse(primitiveData,start,mid,totalNodes);
        newnode->children[1] = recurse(primitiveData,mid  ,end,totalNodes);
        return newnode;
    }
}

bool BVHAccel::Intersect(const Ray &ray, Intersection *isect) const {
    bool hit = IntersectNode(nodes, ray, *isect);
    return hit;
}

bool BVHAccel::IntersectNode(const BVHNode* const node, const Ray &ray, Intersection& isect) const {
    if(node == nullptr) {
        return false;
    }

    float t = -FLT_MAX;
    bool hitnode = node->bounds.Intersect(ray, &t);
//    if(!hitnode || t < 0) { return false; }
    if(!hitnode) { return false; }

    if(node->myprimitives.size() != 0) { //hit leaf
        //no children and has primitives check against all and see which is closest
        Intersection tempisect = Intersection();
        float smallest_t = FLT_MAX;
        bool hit = false;
        for(int i = 0; i < node->myprimitives.size(); ++i){
            std::shared_ptr<Primitive> myprim = node->myprimitives[i];
//            Primitive* myprim = node->myprimitives[i];
            myprim->Intersect(ray,&tempisect);
            if(tempisect.t > 0 && tempisect.t < smallest_t && tempisect.objectHit != nullptr) {
                smallest_t = tempisect.t;
                isect = tempisect;
                hit = true;
            }
        }
        return hit;
    } else {
        Intersection isectleft = Intersection();
        Intersection isectright = Intersection();
        bool hitleft  = IntersectNode(node->children[0], ray, isectleft);
        bool hitright = IntersectNode(node->children[1], ray, isectright);
        if(!hitleft && !hitright) {
            return false;
        } else if(hitleft && !hitright) {
            isect = isectleft;
            return hitleft;
        } else if (!hitleft && hitright) {
            isect = isectright;
            return hitright;
        } else { //hit both
            if(isectleft.t <= isectright.t) {
                isect = isectleft;
                return hitleft;
            } else {
                isect = isectright;
                return hitright;
            }
        }
    }
}

#else
BVHAccel::BVHAccel(const std::vector<std::shared_ptr<Primitive>> &p,
                   int maxPrimsInNode)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)),
      primitives(p) {
    if (primitives.empty()) return;
    // Build BVH from _primitives_
    // Initialize _primitiveInfo_ array for primitives
    std::vector<BVHPrimitiveInfo> primitiveInfo(primitives.size());
    for (size_t i = 0; i < primitives.size(); ++i)
        primitiveInfo[i] = {i, primitives[i]->WorldBound()};
    // Build BVH tree for primitives using _primitiveInfo_
    int totalNodes = 0;
    std::vector<std::shared_ptr<Primitive>> orderedPrims;
    orderedPrims.reserve(primitives.size());
    BVHBuildNode *root;
    root = recursiveBuild(primitiveInfo, 0, primitives.size(),
                              &totalNodes, orderedPrims);
    primitives.swap(orderedPrims);
    // Compute representation of depth-first traversal of BVH tree
    nodes = (LinearBVHNode*)malloc(totalNodes* sizeof *nodes);
    int offset = 0;
    flattenBVHTree(root, &offset);
}

struct BucketInfo {
    int count = 0;
    Bounds3f bounds;
};

BVHBuildNode *BVHAccel::recursiveBuild(
    std::vector<BVHPrimitiveInfo> &primitiveInfo, int start,
    int end, int *totalNodes,
    std::vector<std::shared_ptr<Primitive>> &orderedPrims) {
    BVHBuildNode *node = new BVHBuildNode();
    (*totalNodes)++;
    // Compute bounds of all primitives in BVH node
    Bounds3f bounds;
    for (int i = start; i < end; ++i)
        bounds = Union(bounds, primitiveInfo[i].bounds);
    int nPrimitives = end - start;
    if (nPrimitives == 1) {
        // Create leaf _BVHBuildNode_
        int firstPrimOffset = orderedPrims.size();
        for (int i = start; i < end; ++i) {
            int primNum = primitiveInfo[i].primitiveNumber;
            orderedPrims.push_back(primitives[primNum]);
        }
        node->InitLeaf(firstPrimOffset, nPrimitives, bounds);
        return node;
    } else {
        // Compute bound of primitive centroids, choose split dimension _dim_
        Bounds3f centroidBounds;
        for (int i = start; i < end; ++i)
            centroidBounds = Union(centroidBounds, primitiveInfo[i].centroid);
        int dim = centroidBounds.MaximumExtent();
        // Partition primitives into two sets and build children
        int mid = (start + end) / 2;
        if (centroidBounds.max[dim] == centroidBounds.min[dim]) {
            // Create leaf _BVHBuildNode_
            int firstPrimOffset = orderedPrims.size();
            for (int i = start; i < end; ++i) {
                int primNum = primitiveInfo[i].primitiveNumber;
                orderedPrims.push_back(primitives[primNum]);
            }
            node->InitLeaf(firstPrimOffset, nPrimitives, bounds);
            return node;
        } else {
                // Partition primitives using approximate SAH
                if (nPrimitives <= 4) {
                    // Partition primitives into equally-sized subsets
                    mid = (start + end) / 2;
                    std::nth_element(&primitiveInfo[start], &primitiveInfo[mid],
                                     &primitiveInfo[end - 1] + 1,
                                     [dim](const BVHPrimitiveInfo &a,
                                           const BVHPrimitiveInfo &b) {
                                         return a.centroid[dim] <
                                                b.centroid[dim];
                                     });
                } else {
                    // Allocate _BucketInfo_ for SAH partition buckets
                    constexpr int nBuckets = 12;
                    BucketInfo buckets[nBuckets];
                    // Initialize _BucketInfo_ for SAH partition buckets
                    for (int i = start; i < end; ++i) {
                        int b = nBuckets *
                                centroidBounds.Offset(
                                    primitiveInfo[i].centroid)[dim];
                        if (b == nBuckets) b = nBuckets - 1;
                        buckets[b].count++;
                        buckets[b].bounds =
                            Union(buckets[b].bounds, primitiveInfo[i].bounds);
                    }
                    // Compute costs for splitting after each bucket
                    Float cost[nBuckets - 1];
                    for (int i = 0; i < nBuckets - 1; ++i) {
                        Bounds3f b0, b1;
                        int count0 = 0, count1 = 0;
                        for (int j = 0; j <= i; ++j) {
                            b0 = Union(b0, buckets[j].bounds);
                            count0 += buckets[j].count;
                        }
                        for (int j = i + 1; j < nBuckets; ++j) {
                            b1 = Union(b1, buckets[j].bounds);
                            count1 += buckets[j].count;
                        }
                        cost[i] = 1 +
                                  (count0 * b0.SurfaceArea() +
                                   count1 * b1.SurfaceArea()) /
                                      bounds.SurfaceArea();
                    }
                    // Find bucket to split at that minimizes SAH metric
                    Float minCost = cost[0];
                    int minCostSplitBucket = 0;
                    for (int i = 1; i < nBuckets - 1; ++i) {
                        if (cost[i] < minCost) {
                            minCost = cost[i];
                            minCostSplitBucket = i;
                        }
                    }
                    // Either create leaf or split primitives at selected SAH
                    // bucket
                    Float leafCost = nPrimitives;
                    if (nPrimitives > maxPrimsInNode || minCost < leafCost) {
                        BVHPrimitiveInfo *pmid = std::partition(
                            &primitiveInfo[start], &primitiveInfo[end - 1] + 1,
                            [=](const BVHPrimitiveInfo &pi) {
                                int b = nBuckets *
                                        centroidBounds.Offset(pi.centroid)[dim];
                                if (b == nBuckets) b = nBuckets - 1;
                                return b <= minCostSplitBucket;
                            });
                        mid = pmid - &primitiveInfo[0];
                    } else {
                        // Create leaf _BVHBuildNode_
                        int firstPrimOffset = orderedPrims.size();
                        for (int i = start; i < end; ++i) {
                            int primNum = primitiveInfo[i].primitiveNumber;
                            orderedPrims.push_back(primitives[primNum]);
                        }
                        node->InitLeaf(firstPrimOffset, nPrimitives, bounds);
                        return node;
                    }
                }
             }//end of else

        if(start == mid ||  mid == end) { mid = (start+end)/2;}


            node->InitInterior(dim,
                               recursiveBuild(primitiveInfo, start, mid,
                                              totalNodes, orderedPrims),
                               recursiveBuild(primitiveInfo, mid, end,
                                              totalNodes, orderedPrims));
    }
    return node;
}

int BVHAccel::flattenBVHTree(BVHBuildNode *node, int *offset) {
    LinearBVHNode *linearNode = &nodes[*offset];
    linearNode->bounds = node->bounds;
    int myOffset = (*offset)++;
    if (node->nPrimitives > 0) {
        linearNode->primitivesOffset = node->firstPrimOffset;
        linearNode->nPrimitives = node->nPrimitives;
    } else {
        // Create interior flattened BVH node
        linearNode->axis = node->splitAxis;
        linearNode->nPrimitives = 0;
        flattenBVHTree(node->children[0], offset);
        linearNode->secondChildOffset =
            flattenBVHTree(node->children[1], offset);
    }
    return myOffset;
}

bool BVHAccel::Intersect(const Ray &ray, Intersection *isect) const {
    if (!nodes) return false;
    bool hit = false;
    Vector3f invDir(1 / ray.direction.x, 1 / ray.direction.y, 1 / ray.direction.z);
    int dirIsNeg[3] = {invDir.x < 0, invDir.y < 0, invDir.z < 0};
    // Follow ray through BVH nodes to find primitive intersections
    int toVisitOffset = 0, currentNodeIndex = 0;
    int nodesToVisit[64];

    Intersection min_isect = Intersection();
    float mint = FLT_MAX;

    while (true) {
        const LinearBVHNode *node = &nodes[currentNodeIndex];
        // Check ray against BVH node
        if (node->bounds.IntersectP(ray, invDir, dirIsNeg)) {
            if (node->nPrimitives > 0) {

                // Intersect ray with primitives in leaf BVH node
                for (int i = 0; i < node->nPrimitives; ++i) {

                    if (primitives[node->primitivesOffset + i]->Intersect(ray, &min_isect)) {
                        if(min_isect.t < mint){
                            mint = min_isect.t;
                            *isect = min_isect;
                            hit = true;
                        }
                    }
                }
                if (toVisitOffset == 0) break;
                currentNodeIndex = nodesToVisit[--toVisitOffset];
            } else {
                // Put far BVH node on _nodesToVisit_ stack, advance to near
                // node
                if (dirIsNeg[node->axis]) {
                    nodesToVisit[toVisitOffset++] = currentNodeIndex + 1;
                    currentNodeIndex = node->secondChildOffset;
                } else {
                    nodesToVisit[toVisitOffset++] = node->secondChildOffset;
                    currentNodeIndex = currentNodeIndex + 1;
                }
            }
        } else {
            if (toVisitOffset == 0) break;
            currentNodeIndex = nodesToVisit[--toVisitOffset];
        }
    }
    return hit;
}

#endif
