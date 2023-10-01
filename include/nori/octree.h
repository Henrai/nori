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
    struct Node;
    using NodePtr = std::unique_ptr<Node>;
    struct Node {
        BoundingBox3f box;
        std::vector<NodePtr> children;
        std::vector<std::pair<uint8_t, uint32_t>> ids;
        bool traverse(Ray3f& ray, Intersection& it, uint32_t& idx, bool shadowRay, const std::vector<Mesh*>& meshes, bool shouldSort) const;
    };
    void appendMesh(Mesh* mesh);
    void build();
    NodePtr build(int depth, const BoundingBox3f& bbox, const std::vector<std::pair<uint8_t, uint32_t>>& triangles);
    bool traverse(Ray3f& ray, Intersection& it, uint32_t& idx, bool shadowRay) const;


    int max_depth = 0;
    int node_cnt = 0;
    int leaf_cnt = 0;
    int triangle_cnt = 0;
    friend Node;
private:
    NodePtr root;
    static constexpr int MAX_NODE = 10;
    static constexpr int MAX_DEPTH = 8;
    bool shouldSort = true;
    std::vector<Mesh*> meshes_;
    BoundingBox3f boundingBox_;
    std::vector<std::pair<uint8_t, uint32_t>> triangles_;
    
};
NORI_NAMESPACE_END
