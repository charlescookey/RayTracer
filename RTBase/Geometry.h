#pragma once

#include "Core.h"
#include "Sampling.h"

class Ray
{
public:
	Vec3 o;
	Vec3 dir;
	Vec3 invDir;
	Ray()
	{
	}
	Ray(Vec3 _o, Vec3 _d)
	{
		init(_o, _d);
	}
	void init(Vec3 _o, Vec3 _d)
	{
		o = _o;
		dir = _d;
		invDir = Vec3(1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z);
	}
	Vec3 at(const float t) const
	{
		return (o + (dir * t));
	}
};

class Plane
{
public:
	Vec3 n;
	float d;
	void init(Vec3& _n, float _d)
	{
		n = _n;
		d = _d;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{
		t = (d - Dot(n, r.o)) / Dot(n, r.dir);
		return t >= 0.f;
	}
};

#define EPSILON 0.001f
#define M_EPSILON 0.00001f

class Triangle
{
public:
	Vertex vertices[3];
	Vec3 e1; // Edge 1
	Vec3 e2; // Edge 2
	Vec3 n; // Geometric Normal

	Vec3 em1;
	Vec3 em2;
	Vec3 nPrime;
	float area; // Triangle area
	float d; // For ray triangle if needed
	unsigned int materialIndex;
	void init(Vertex v0, Vertex v1, Vertex v2, unsigned int _materialIndex)
	{
		materialIndex = _materialIndex;
		vertices[0] = v0;
		vertices[1] = v1;
		vertices[2] = v2;
		e1 = vertices[2].p - vertices[1].p;
		e2 = vertices[0].p - vertices[2].p;

		em1 = vertices[1].p - vertices[0].p;
		em2 = vertices[2].p - vertices[0].p;

		n = e1.cross(e2).normalize();
		nPrime = e1.cross(e2);
		area = e1.cross(e2).length() * 0.5f;
		d = Dot(n, vertices[0].p);
	}
	Vec3 centre() const
	{
		return (vertices[0].p + vertices[1].p + vertices[2].p) / 3.0f;
	}
	//Add code here
	bool rayIntersect3(const Ray& r, float& t, float& u, float& v) const
	{
		//u,v are barycentric
		t = (d - Dot(n, r.o)) / Dot(n, r.dir);
		if (t < 0.f)return false;

		Vec3 P = r.at(t);
		Vec3 q1 = P - vertices[1].p;
		Vec3 C1 = Cross(e1, q1);

		float area2 = area * 2;

		u = Dot(C1, n) / area2;
		if (u > 1.f || u < 0.f)return false;

		Vec3 q2 = P - vertices[2].p;
		Vec3 C2 = Cross(e2, q2);

		v = Dot(C2, n) / area2;
		if (v > 1.f || v < 0.f)return false;

		if (u + v > 1.f)return false;


		return true;
	}

	bool rayIntersect(const Ray& r, float& t, float& u, float& v) const
	{
		float denom = Dot(n, r.dir);
		if (denom == 0) { return false; }
		t = (d - Dot(n, r.o)) / denom;
		if (t < 0) { return false; }
		Vec3 p = r.at(t);
		float invArea = 1.0f / Dot(e1.cross(e2), n);
		u = Dot(e1.cross(p - vertices[1].p), n) * invArea;
		if (u < 0 || u > 1.0f) { return false; }
		v = Dot(e2.cross(p - vertices[2].p), n) * invArea;
		if (v < 0 || (u + v) > 1.0f) { return false; }
		return true;
	}

	//finish muller trumbone, use smallewr epsilon
	bool rayIntersect2(const Ray& r, float& t, float& u, float& v) const
	{
		Vec3 p = Cross(r.dir, em2);
		float det = Dot(em1, p);


		if (std::abs(det) < M_EPSILON)return false;

		float invdet = 1.0f / det;

		Vec3 T = r.o - vertices[0].p;
		u = Dot(T, p) * invdet;
		if (u < 0.0f || u > 1.0f) { return false; }

		Vec3 q = Cross(T, em1);
		v = Dot(r.dir, q) * invdet;
		if (v < 0.0f || (u + v) > 1.0f) { return false; }

		t = Dot(em2, q) * invdet;

		return t >= 0.0f;
	}

	void interpolateAttributes(const float alpha, const float beta, const float gamma, Vec3& interpolatedNormal, float& interpolatedU, float& interpolatedV) const
	{
		interpolatedNormal = vertices[0].normal * alpha + vertices[1].normal * beta + vertices[2].normal * gamma;
		interpolatedNormal = interpolatedNormal.normalize();
		interpolatedU = vertices[0].u * alpha + vertices[1].u * beta + vertices[2].u * gamma;
		interpolatedV = vertices[0].v * alpha + vertices[1].v * beta + vertices[2].v * gamma;
	}
	// Add code here
	Vec3 sample(Sampler* sampler, float& pdf)
	{
		float r1 = sampler->next();
		float r2 = sampler->next();

		float alpha = 1 - std::sqrt(r1);
		float beta = r2 * std::sqrt(r1);
		float gamma = 1 - (alpha + beta);

		pdf = 1 / area;

		return Vec3((vertices[0].p * alpha) + (vertices[1].p * beta) + (vertices[2].p * gamma));
	}
	Vec3 gNormal()
	{
		return (n * (Dot(vertices[0].normal, n) > 0 ? 1.0f : -1.0f));
	}
};

class AABB
{
public:
	Vec3 max;
	Vec3 min;
	AABB()
	{
		reset();
	}
	void reset()
	{
		max = Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		min = Vec3(FLT_MAX, FLT_MAX, FLT_MAX);
	}
	void extend(const Vec3 p)
	{
		max = Max(max, p);
		min = Min(min, p);
	}

	void extend(const AABB a)
	{
		extend(a.max);
		extend(a.min);
	}
	// Add code here
	bool rayAABB(const Ray& r, float& t)
	{
		Vec3 tmin = (min - r.o) * r.invDir;
		Vec3 tmax = (max - r.o) * r.invDir;
		Vec3 Tentry = Min(tmin, tmax);
		Vec3 Texit = Max(tmin, tmax);
		float tentry = (std::max)(Tentry.x, (std::max)(Tentry.y, Tentry.z));
		float texit = (std::min)(Texit.x, (std::min)(Texit.y, Texit.z));
		t = (std::min)(tentry, texit);
		return (tentry <= texit);
	}
	// Add code here
	bool rayAABB(const Ray& r)
	{
		Vec3 tmin = (min - r.o) * r.invDir;
		Vec3 tmax = (max - r.o) * r.invDir;
		Vec3 Tentry = Min(tmin, tmax);
		Vec3 Texit = Max(tmin, tmax);
		float tentry = (std::max)(Tentry.x, (std::max)(Tentry.y, Tentry.z));
		float texit = (std::min)(Texit.x, (std::min)(Texit.y, Texit.z));

		return (tentry <= texit);
	}
	// Add code here
	float area()
	{
		Vec3 size = max - min;
		return ((size.x * size.y) + (size.y * size.z) + (size.x * size.z)) * 2.0f;
	}
};


class Sphere
{
public:
	Vec3 centre;
	float radius;
	void init(Vec3& _centre, float _radius)
	{
		centre = _centre;
		radius = _radius;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{
		return false;
	}
};

struct IntersectionData
{
	unsigned int ID;
	float t;
	float alpha;
	float beta;
	float gamma;
};

#define MAXNODE_TRIANGLES 8
#define TRAVERSE_COST 1.0f
#define TRIANGLE_COST 2.0f
#define BUILD_BINS 32


#define BINS_COUNT 8

const float C_bounds = 1.0f; // Cost of traversing a node
const float C_isect = 1.0f;  // Cost of a ray-triangle intersection


struct Bin {
	AABB bounds;
	int triCount = 0;
};

#include <unordered_map>
std::unordered_map<Triangle*, int> triangleMap;
int maxDepth = 0;

int maxNodeTri = 0;
int nodeAbove = 0;
int maxCheckSize = 8;

class BVHNode
{
public:
	AABB bounds;
	BVHNode* r;
	BVHNode* l;
	// This can store an offset and number of triangles in a global triangle list for example
	// But you can store this however you want!
	//unsigned int offset;
	int num;
	std::vector<Triangle*> triangles;
	int maxD() {
		return maxDepth;
	}
	BVHNode()
	{
		r = NULL;
		l = NULL;
	}
	// Note there are several options for how to implement the build method. Update this as required
	void build(std::vector<Triangle>& inputTriangles) {
		std::vector<Triangle*> TempTriangles;
		for (int i = 0; i < inputTriangles.size(); i++) {
			TempTriangles.push_back(&inputTriangles[i]);
			triangleMap[&inputTriangles[i]] = i;
		}
		RecursiveBuild(TempTriangles);
	}

	float findBestSplitPlane(int& axis, float& splitPos) {
		float bestCost = FLT_MAX;
		float parentArea = bounds.area();

		for (int a = 0; a < 3; a++) {
			float boundsMin = FLT_MAX;
			float boundsMax = -FLT_MAX;

			for (Triangle* tri : triangles) {
				float centroidPos = tri->centre().coords[a];
				boundsMin = std::min(boundsMin, centroidPos);
				boundsMax = std::max(boundsMax, centroidPos);
			}

			if (boundsMin == boundsMax) continue;

			Bin bin[BINS_COUNT];
			float scale = BINS_COUNT / (boundsMax - boundsMin);

			for (Triangle* tri : triangles) {
				float centroidPos = tri->centre().coords[a];
				int binIdx = (std::min)(BINS_COUNT - 1, static_cast<int>((centroidPos - boundsMin) * scale));
				bin[binIdx].triCount++;
				bin[binIdx].bounds.extend(tri->vertices[0].p);
				bin[binIdx].bounds.extend(tri->vertices[1].p);
				bin[binIdx].bounds.extend(tri->vertices[2].p);
			}

			// Gather data 7 planes between 8 bins
			float leftArea[BINS_COUNT - 1], rightArea[BINS_COUNT - 1];
			int leftCount[BINS_COUNT - 1], rightCount[BINS_COUNT - 1];
			AABB leftBox, rightBox;
			int leftSum = 0, rightSum = 0;

			for (int i = 0; i < BINS_COUNT - 1; i++) {
				leftSum += bin[i].triCount;
				leftCount[i] = leftSum;
				if (bin[i].triCount > 0) {
					leftBox.extend(bin[i].bounds);
					leftArea[i] = leftBox.area();
				}
				else leftArea[i] = 0;

				rightSum += bin[(BINS_COUNT - 1) - i].triCount;
				rightCount[(BINS_COUNT - 2) - i] = rightSum;
				if (bin[(BINS_COUNT - 1) - i].triCount > 0) {
					rightBox.extend(bin[(BINS_COUNT - 1) - i].bounds);
					rightArea[(BINS_COUNT - 2) - i] = rightBox.area();
				}
				else rightArea[(BINS_COUNT - 2) - i] = 0;
			}

			float BinScale = (boundsMax - boundsMin) / BINS_COUNT;
			for (int i = 0; i < BINS_COUNT - 1; i++) {
				float cost = C_bounds + ((leftArea[i] / parentArea) * leftCount[i] * C_isect) + ((rightArea[i] / parentArea) * rightCount[i] * C_isect);
				if (cost < bestCost) {
					bestCost = cost;
					axis = a;
					splitPos = boundsMin + BinScale * (i + 1);
				}
			}
		}
		return bestCost;
	}

	float calculateNodeCost() {
		float area = bounds.area();
		return num * area;
	}

	void RecursiveBuild(std::vector<Triangle*>& inputTriangles)
	{
		// Add BVH building code here
		//change later to not have traibge pointer voetor
		num = inputTriangles.size();
		triangles.reserve(num);
		for (Triangle* a : inputTriangles) {
			triangles.emplace_back(a);
		}

		if (num <= 1)return;

		for (Triangle* a : triangles) {
			bounds.extend(a->vertices[0].p);
			bounds.extend(a->vertices[1].p);
			bounds.extend(a->vertices[2].p);
		}

		int axis = -1;
		float splitPos = 0.0f;
		float splitCost = findBestSplitPlane(axis, splitPos);
		float noSplitCost = calculateNodeCost();

		//if (splitCost >= noSplitCost || axis == -1) return;

		std::vector<Triangle*> leftTriangles, rightTriangles;
		for (Triangle* tri : triangles) {
			if (tri->centre().coords[axis] < splitPos) {
				leftTriangles.push_back(tri);
			}
			else {
				rightTriangles.push_back(tri);
			}
		}

		if (leftTriangles.empty() || rightTriangles.empty()) return;
		
		triangles.clear();

		l = new BVHNode();
		r = new BVHNode();

		// Recursively build children
		l->RecursiveBuild(leftTriangles);
		r->RecursiveBuild(rightTriangles);


	}

	void traverse(const Ray& ray, const std::vector<Triangle>& InputTriangles, IntersectionData& intersection)
	{
		// Add BVH Traversal code here
		if (!bounds.rayAABB(ray)) return;

		if (l == nullptr && r == nullptr) {
			for (Triangle* tri : triangles) {
				float t, u, v;
				if (tri->rayIntersect(ray, t, u, v) && t < intersection.t) {
					intersection.t = t;
					intersection.alpha = u;
					intersection.beta = v;
					intersection.gamma = 1.0f - (u + v);
					intersection.ID = triangleMap[tri];

				}
			}
			return;
		}

		if (l) l->traverse(ray, InputTriangles, intersection);
		if (r) r->traverse(ray, InputTriangles, intersection);
	}

	IntersectionData traverse(const Ray& ray, const std::vector<Triangle>& triangles)
	{
		IntersectionData intersection;
		intersection.t = FLT_MAX;
		traverse(ray, triangles, intersection);
		return intersection;
	}
	bool traverseVisible(const Ray& ray, const std::vector<Triangle>& InputTriangles, const float maxT)
	{
		// Add visibility code here
		float t, u, v;
		if (!bounds.rayAABB(ray, t) || t > maxT) return true;

		if (l == nullptr && r == nullptr) {
			for (Triangle* tri : triangles) {
				if (tri->rayIntersect(ray, t, u, v) && t < maxT) {
					return false;
				}
			}
			return true;
		}

		if (l && !l->traverseVisible(ray, InputTriangles, maxT)) return false;
		if (r && !r->traverseVisible(ray, InputTriangles, maxT)) return false;
		return true;
	}

	//FOR DEBUGGING
	void print(int depth = 0) {
		// Indentation for visualizing hierarchy
		std::string indent(depth * 2, ' ');

		// Print node information
		std::cout << indent << (l == nullptr && r == nullptr ? "Leaf" : "Node") << " (num triangles: " << static_cast<int>(num) << ")\n";
		std::cout << indent << "Bounds: min=(" << bounds.min.x << ", " << bounds.min.y << ", " << bounds.min.z << "), "
			<< "max=(" << bounds.max.x << ", " << bounds.max.y << ", " << bounds.max.z << ")\n";

		// Print triangles if this is a leaf node
		if (l == nullptr && r == nullptr) {
			for (Triangle* a : triangles) {
				std::cout << indent << "  Triangle " << triangleMap[a] << ": "
					<< "v0=(" << a->vertices[0].p.x << ", " << a->vertices[0].p.y << ", " << a->vertices[0].p.z << "), "
					<< "v1=(" << a->vertices[1].p.x << ", " << a->vertices[1].p.y << ", " << a->vertices[1].p.z << "), "
					<< "v2=(" << a->vertices[2].p.x << ", " << a->vertices[2].p.y << ", " << a->vertices[2].p.z << ")\n";
			}
		}

		// Recursively print children
		if (l) {
			std::cout << indent << "Left child:\n";
			l->print(depth + 1);
		}
		if (r) {
			std::cout << indent << "Right child:\n";
			r->print(depth + 1);
		}
	}
	void print2(int depth = 0) {
		// Indentation for visualizing hierarchy
		maxDepth = (std::max)(maxDepth, depth);
		std::string indent(depth * 2, '-');

		// Print node information
		//std::cout << indent << (l == nullptr && r == nullptr ? "Leaf" : "Node") << " (num triangles: " << static_cast<int>(num) << ")\n";

		// Print triangles if this is a leaf node
		if (l == nullptr && r == nullptr) {
			for (Triangle* a : triangles) {
				//std::cout << indent << "  Triangle " << triangleMap[a] << "\n ";
			}
		}

		// Recursively print children
		if (l) {
			//std::cout << indent << "Left child:\n";
			l->print2(depth + 1);
		}
		if (r) {
			//std::cout << indent << "Right child:\n";
			r->print2(depth + 1);
		}
	}


	void printStat() {
		std::cout << "maxdepth: " << maxD() << "\n";
		std::cout << "Max triangles in a node: " << maxNodeTri << "\n";
		std::cout << "Number of nodes with more than " << maxCheckSize << " triangles: " << nodeAbove << "\n";
	}

	void checkTraverse()
	{
		if (l == nullptr && r == nullptr) {
			if (triangles.size() > 1) {
				maxNodeTri = std::max(maxNodeTri, (int)triangles.size());
			}
			if (triangles.size() > maxCheckSize) {
				nodeAbove++;
			}
			return;
		}

		if (l) l->checkTraverse();
		if (r) r->checkTraverse();
	}
};
