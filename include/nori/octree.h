#pragma once 

#include "common.h"
#include "bbox.h"
#include "mesh.h"
//
// Created by waz on 8/29/2023.
//

NORI_NAMESPACE_BEGIN

class Octree {
public:
    BoundingBox3f box;
    std::vector<Octree*> children;
    std::vector<uint32_t> ids;

    static Octree* build(Mesh* mesh);

    static Octree* build(const BoundingBox3f& bbox, std::vector<uint32_t>& triangles, Mesh* mesh);

    bool traverse(Ray3f& ray, Intersection& it, uint32_t& idx, bool shadowRay, Mesh* mesh);
};
NORI_NAMESPACE_END
