//
// Created by waz on 8/30/2023.
//

#include <nori/octree.h>
#include <nori/mesh.h>
#include <numeric>
#include <array>
#include <iostream>
#include <chrono>
#include <vector>
NORI_NAMESPACE_BEGIN
void Octree::build(Mesh* mesh) {
    if (mesh == nullptr) {
        root = nullptr;
        return;
    }
    uint32_t triangleCnt = mesh->getTriangleCount();
    std::vector<uint32_t> triangles(triangleCnt);
    std::iota(triangles.begin(), triangles.end(), 0);
    root = build(1, mesh->getBoundingBox(), triangles, mesh);
}

Octree::NodePtr Octree::build(int depth, const BoundingBox3f& bbox, const std::vector<uint32_t>& triangles, Mesh* mesh) {

    if (depth > max_depth) max_depth = depth;
    NodePtr node = std::make_unique<Node>();
    node->box = bbox;
    node->children = std::vector<NodePtr>(0);
    node->ids = triangles;
    
    if (triangles.size() < MAX_NODE || depth == MAX_DEPTH) {
        leaf_cnt++;
        triangle_cnt += triangles.size();
        return node;
    }
    node_cnt++;
    node->children.resize(8);
    auto minP = bbox.min, maxP = bbox.max;
    auto midP = bbox.getCenter();
    std::array<float, 3> child_min{}, child_max{};
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 3; j++) {
            child_min[j] = (i & (1 << j)) ? midP(j) : minP(j);
            child_max[j] = (i & (1 << j)) ? maxP(j) : midP(j);
        }
        BoundingBox3f child_box = BoundingBox3f(
            Point3f(child_min[0], child_min[1], child_min[2]),
            Point3f(child_max[0], child_max[1], child_max[2]));
        std::vector<uint32_t> triangleId;
        const MatrixXu& F = mesh->getIndices();
        const MatrixXf& V = mesh->getVertexPositions();

        for (auto id : triangles) {
            uint32_t idx0 = F(0, id), idx1 = F(1, id), idx2 = F(2, id);
            Point3f p0 = V.col(idx0), p1 = V.col(idx1), p2 = V.col(idx2);
            BoundingBox3f tri_box(p0);
            tri_box.expandBy(p1);
            tri_box.expandBy(p2);
            if (child_box.overlaps(tri_box)) triangleId.push_back(id);
        }
        node->children[i] = build(depth + 1, child_box, triangleId, mesh);
           
    }

    return node;
}

bool Octree::traverse(Ray3f& ray, Intersection& it, uint32_t& idx, bool shadowRay, Mesh* mesh) const
{
    return root->traverse(ray, it, idx, shadowRay, mesh, shouldSort);
}

bool Octree::Node::traverse(Ray3f & ray, Intersection & it, uint32_t & idx, bool shadowRay, Mesh * mesh, bool shouldSort) const {
    if (!box.rayIntersect(ray)) return false;
    bool isHit= false;
    if (children.size() == 0) {
        for (auto id : ids) {
            float u, v, t;
            if (mesh->rayIntersect(id, ray, u, v, t) && t <= ray.maxt) {
                if (shadowRay)
                    return true;
                 ray.maxt = it.t = t;
                 it.uv = Point2f(u, v);
                 it.mesh = mesh;
                 idx = id;
                 isHit = true;
            } 
        }
    }
    else {
        if (shouldSort) {
            std::pair<float, size_t> distances[8] = {};
            for (int i = 0; i < 8; i++) {
                float distance = children[i]->box.distanceTo(ray.o);
                distances[i] = std::move(std::make_pair(distance, i));
            }

            std::sort(distances, distances + 8, [](const auto& l, const auto& r) { return l.first < r.first; });

            for (auto pair : distances) {
                float distance = pair.first;
                Node* child = children[pair.second].get();
                if (distance > ray.maxt) break;
                isHit |= child->traverse(ray, it, idx, shadowRay, mesh, shouldSort);
                if (shadowRay && isHit) return true;
            }
        }
        else {
            for (const auto& child : children) {
                isHit |= child->traverse(ray, it, idx, shadowRay, mesh, shouldSort);
                if (shadowRay && isHit) return true;
            }
        }
    }
    return isHit;
}



NORI_NAMESPACE_END