#pragma once

#include <nori/bbox.h>
#include <nori/mesh.h>

NORI_NAMESPACE_BEGIN

struct Node {
	Node* children[8];
	BoundingBox3f boundingBox;
	std::vector<uint32_t> faceIndices;

	const Node* rayIntersect(const Ray3f& ray) const;
};

Node* build(BoundingBox3f box, const MatrixXf& triangles, const MatrixXu& faces, const std::vector<uint32_t>& faceIndices = std::vector<uint32_t>());

NORI_NAMESPACE_END