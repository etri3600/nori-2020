#include <nori/octree.h>

NORI_NAMESPACE_BEGIN

const Node* Node::rayIntersect(const Ray3f& ray) const
{
	if (boundingBox.rayIntersect(ray))
	{
		for (Node* child : children)
		{
			if (child)
			{
				const Node* intersectNode = child->rayIntersect(ray);
				if (intersectNode)
				{
					return intersectNode;
				}
			}
		}

		return this;
	}

	return nullptr;
}

Node* build(BoundingBox3f box, const MatrixXf& triangles, const MatrixXu& faces, const std::vector<uint32_t>& faceIndices) {
	if (triangles.cols() == 0)
		return nullptr;

	if (box.getVolume() < 4.0f || (0 < faceIndices.size() && faceIndices.size() <= 16))
	{
		if (faceIndices.size() > 0)
		{
			Node* node = new Node();
			node->boundingBox = box;
			node->faceIndices = faceIndices;
			return node;
		}
		return nullptr;
	}

	std::vector<uint32_t> list[8];

	Vector3f extent = box.getExtents();
	Point3f min = box.min;
	Point3f max = box.max;
	Point3f mid = min + extent * 0.5f;

	BoundingBox3f subbox[8];
	subbox[0] = BoundingBox3f(min, mid);
	subbox[1] = BoundingBox3f(Point3f(mid[0], min[1], min[2]), Point3f(max[0], mid[1], mid[2]));
	subbox[2] = BoundingBox3f(Point3f(min[0], mid[1], min[2]), Point3f(mid[0], max[1], mid[2]));
	subbox[3] = BoundingBox3f(Point3f(mid[0], mid[1], min[2]), Point3f(max[0], max[1], mid[2]));

	subbox[4] = BoundingBox3f(Point3f(min[0], min[1], mid[2]), Point3f(mid[0], mid[1], max[2]));
	subbox[5] = BoundingBox3f(Point3f(mid[0], min[1], mid[2]), Point3f(max[0], mid[1], max[2]));
	subbox[6] = BoundingBox3f(Point3f(min[0], mid[1], mid[2]), Point3f(mid[0], max[1], max[2]));
	subbox[7] = BoundingBox3f(mid, max);

	bool useFaceIndices = faceIndices.size() > 0;

	uint32_t faceCount = useFaceIndices ? static_cast<uint32_t>(faceIndices.size()) : static_cast<uint32_t>(faces.cols());
	auto faceColumnIndexSelectr = [useFaceIndices, &faceIndices](uint32_t index) -> uint32_t
	{
		if (useFaceIndices)
		{
			return faceIndices[index];
		}
		else
		{
			return index;
		}
	};

	for (uint32_t i = 0; i < faceCount; ++i)
	{
		Vector3f vert0 = triangles.col(faces(0, faceColumnIndexSelectr(i)));
		Vector3f vert1 = triangles.col(faces(1, faceColumnIndexSelectr(i)));
		Vector3f vert2 = triangles.col(faces(2, faceColumnIndexSelectr(i)));
		for (uint32_t j = 0; j < 8; ++j)
		{
			if (subbox[j].contains(vert0) || subbox[j].contains(vert1) || subbox[j].contains(vert2))
			{
				list[j].push_back(i);
			}
		}
	}

	Node* node = new Node();
	node->boundingBox = box;
	bool bLeaf = true;
	for (int i = 0; i < 8; ++i)
	{
		if (list[i].size() > 0)
		{
			node->children[i] = build(subbox[i], triangles, faces, list[i]);
			if (node->children[i] != nullptr)
				bLeaf &= false;
		}
		else
			node->children[i] = nullptr;
	}

	if (bLeaf)
		node->faceIndices = faceIndices;

	return node;
}

NORI_NAMESPACE_END