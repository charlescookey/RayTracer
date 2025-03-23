#pragma once

#include "Core.h"
#include <random>
#include <algorithm>

class Sampler
{
public:
	virtual float next() = 0;
};


//onr per thread, seed diuff for diff thread, one mark
class MTRandom : public Sampler
{
public:
	std::mt19937 generator;
	std::uniform_real_distribution<float> dist;
	MTRandom(unsigned int seed = 1) : dist(0.0f, 1.0f)
	{
		generator.seed(seed);
	}
	float next()
	{
		return dist(generator);
	}
};

const float RAdius = 1.f;
// Note all of these distributions assume z-up coordinate system
class SamplingDistributions
{
public:
	static Vec3 uniformSampleHemisphere(float r1, float r2)
	{
		// Add code here
		float theta = acos(r1);
		float phi = 2 * M_PI * r2;

		float x = RAdius * sin(theta) * cos(phi);
		float y = RAdius * sin(theta) * sin(phi);
		float z = RAdius * cos(theta);

		return Vec3(x, y, z);
	}
	static float uniformHemispherePDF(const Vec3 wi)
	{
		// Add code here
		return 1.0f / 2 * M_PI;
	}
	static Vec3 cosineSampleHemisphere(float r1, float r2)
	{
		// Add code here
		float theta = acos(sqrtf(r1));
		float phi = 2 * M_PI * r2;

		float x = RAdius * sin(theta) * cos(phi);
		float y = RAdius * sin(theta) * sin(phi);
		float z = RAdius * cos(theta);

		return Vec3(x, y, z);;
	}
	static float cosineHemispherePDF(const Vec3 wi)
	{
		// Add code here
		return wi.z / M_PI;
	}
	static Vec3 uniformSampleSphere(float r1, float r2)
	{
		// Add code here
		float theta = acos(1 - (2 * r1));
		float phi = 2 * M_PI * r2;

		float x = RAdius * sin(theta) * cos(phi);
		float y = RAdius * sin(theta) * sin(phi);
		float z = RAdius * cos(theta);

		return Vec3(x, y, z);
	}
	static float uniformSpherePDF(const Vec3& wi)
	{
		// Add code here
		return 1.0f / 4 * M_PI;
	}
};