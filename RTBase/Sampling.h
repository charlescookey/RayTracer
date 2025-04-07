#pragma once

#include "Core.h"
#include <random>
#include <algorithm>
#include <memory>

class Sampler
{
public:
	virtual float next() = 0;
};

namespace HelperFunctions
{
	template<typename T>
	T clamp(T num, T low, T high) {
		return std::max(low, std::min(num, high));
	}
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
//1/64
#define PSSs1 0.015625f 
//1/1024
#define PSSs2 0.0009765625f

//The priamiary space state
class PSS {
public:
	std::vector<float> pssCoords;
	std::vector<float> pssTimer;
	MTRandom random;

	int currentIndex;

	PSS() : currentIndex(0) { }

	void reset()
	{
		currentIndex = 0;
		pssCoords.clear();
		pssTimer.clear();
	}

	float next() {
		if (currentIndex >= pssCoords.size()) {
			pssCoords.push_back(random.next());
			pssTimer.push_back(0);
		}

		return pssCoords[currentIndex++];
	}

	//mutate the value of the coordinate
	float mutate(float input)
	{
		float r1 = random.next();
		float r2 = random.next();
		float delta = PSSs1 * std::exp(-std::log(PSSs1 / PSSs2) * r1);

		if (r2 < 0.5f)
		{
			delta = -delta;
		}

		return std::fmod(input + delta + 1.0f, 1.0f); // wrap around
	}

	//when a small step is taken
	void smallStepPerturb(int timer) {
		for (int i = 0; i < pssCoords.size(); i++)
		{
			while (pssTimer[i] < timer)
			{
				pssCoords[i] = mutate(pssCoords[i]);
				pssTimer[i]++;
			}
		}
	}

};

class PSSSampler : public Sampler
{
public:
	int currentTimer;
	PSS currentState;
	PSS proposedState;
	MTRandom random;


	PSSSampler(unsigned int seed = 1)
	{
		
		currentTimer = 0;
	}

	void reset()
	{
		currentTimer = 0;
	}

	void largeStep()
	{
		currentTimer++;
		proposedState.reset();
	}

	float nextPSSP() {
		return proposedState.next();
	}

	float nextPSSC() {
		return currentState.next();
	}

	float next() {
		return random.next();
	}

	void smallStep() {
		proposedState.smallStepPerturb(currentTimer);
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

struct Vec2 {
	float x, y;
	Vec2() : x(0), y(0) {}
	Vec2(float x, float y) : x(x), y(y) {}

	int operator[](int index) const
	{
		if (index == 0)
			return x;
		return y;
	}
};
