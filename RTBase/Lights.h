#pragma once

#include "Core.h"
#include "Geometry.h"
#include "Materials.h"
#include "Sampling.h"

#include <numeric>

#pragma warning( disable : 4244)

class SceneBounds
{
public:
	Vec3 sceneCentre;
	float sceneRadius;
};

class Light
{
public:
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf) = 0;
	virtual Colour evaluate(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isArea() = 0;
	virtual Vec3 normal(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float totalIntegratedPower() = 0;
	virtual Vec3 samplePositionFromLight(Sampler* sampler, float& pdf) = 0;
	virtual Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf) = 0;
};

class AreaLight : public Light
{
public:
	Triangle* triangle = NULL;
	Colour emission;
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf)
	{
		emittedColour = emission;
		return triangle->sample(sampler, pdf);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		if (Dot(wi, triangle->gNormal()) < 0)
		{
			return emission;
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return 1.0f / triangle->area;
	}
	bool isArea()
	{
		return true;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return triangle->gNormal();
	}
	float totalIntegratedPower()
	{
		return (triangle->area * emission.Lum());
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		return triangle->sample(sampler, pdf);
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		// Add code to sample a direction from the light
		Vec3 wi = Vec3(0, 0, 1);
		pdf = 1.0f;
		Frame frame;
		frame.fromVector(triangle->gNormal());
		return frame.toWorld(wi);
	}
};

class BackgroundColour : public Light
{
public:
	Colour emission;
	BackgroundColour(Colour _emission)
	{
		emission = _emission;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		reflectedColour = emission;
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		return emission;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return SamplingDistributions::uniformSpherePDF(wi);
	}
	bool isArea()
	{
		return false;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return -wi;
	}
	float totalIntegratedPower()
	{
		return emission.Lum() * 4.0f * M_PI;
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 p = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		p = p * use<SceneBounds>().sceneRadius;
		p = p + use<SceneBounds>().sceneCentre;
		pdf = 4 * M_PI * use<SceneBounds>().sceneRadius * use<SceneBounds>().sceneRadius;
		return p;
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		return wi;
	}
};

class EnvironmentMap : public Light
{
public:
	Texture* env;
	EnvironmentMap(Texture* _env)
	{
		env = _env;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Assignment: Update this code to importance sampling lighting based on luminance of each pixel
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		reflectedColour = evaluate(shadingData, wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		float u = atan2f(wi.z, wi.x);
		u = (u < 0.0f) ? u + (2.0f * M_PI) : u;
		u = u / (2.0f * M_PI);
		float v = acosf(wi.y) / M_PI;
		return env->sample(u, v);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Assignment: Update this code to return the correct PDF of luminance weighted importance sampling
		return SamplingDistributions::uniformSpherePDF(wi);
	}
	bool isArea()
	{
		return false;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return -wi;
	}
	float totalIntegratedPower()
	{
		float total = 0;
		for (int i = 0; i < env->height; i++)
		{
			float st = sinf(((float)i / (float)env->height) * M_PI);
			for (int n = 0; n < env->width; n++)
			{
				total += (env->texels[(i * env->width) + n].Lum() * st);
			}
		}
		total = total / (float)(env->width * env->height);
		return total * 4.0f * M_PI;
	}




	std::vector<float> luminance_cdf;
	std::vector<float> marginal_cdf;
	std::vector<std::vector<float>> conditional_cdf;


	void buildDistributions() {
		if (!env || !env->texels) return;

		int width = env->width;
		int height = env->height;

		luminance_cdf.resize(width * height);
		marginal_cdf.resize(height);
		conditional_cdf.resize(height, std::vector<float>(width));


		std::vector<float> table(width * height);
		for (int v = 0; v < height; v++) {
			for (int u = 0; u < width; u++) {
				int idx = v * width + u;
				table[idx] = env->texels[idx].Lum();
			}
		}

		std::vector<float> row_sums(height, 0.0f);
		for (int v = 0; v < height; v++) {
			for (int u = 0; u < width; u++) {
				row_sums[v] += table[v * width + u];
			}
		}

		float total_sum = std::accumulate(row_sums.begin(), row_sums.end(), 0.0f);
		if (total_sum > 0) {
			float cumulative = 0.0f;
			for (int v = 0; v < height; v++) {
				cumulative += row_sums[v] / total_sum;
				marginal_cdf[v] = cumulative;
			}
		}

		for (int v = 0; v < height; v++) {
			if (row_sums[v] > 0) {
				float cumulative = 0.0f;
				for (int u = 0; u < width; u++) {
					int idx = v * width + u;
					cumulative += table[idx] / row_sums[v];
					conditional_cdf[v][u] = cumulative;
				}
			}
		}

		float cumulative = 0.0f;
		for (int i = 0; i < width * height; i++) {
			cumulative += table[i] / total_sum;
			luminance_cdf[i] = cumulative;
		}
	}

	Vec3 sampleImportance(Sampler* sampler, float& pdf) {
		float xi = sampler->next();
		float yi = sampler->next();

		int v = 0;
		while (v < env->height - 1 && marginal_cdf[v] < yi) {
			v++;
		}

		int u = 0;
		while (u < env->width - 1 && conditional_cdf[v][u] < xi) {
			u++;
		}

		float u0 = (float)u / (float)env->width;
		float v0 = (float)v / (float)env->height;
		float u1 = (float)(u + 1) / (float)env->width;
		float v1 = (float)(v + 1) / (float)env->height;

		float u_ = u0 + (u1 - u0) * sampler->next();
		float v_ = v0 + (v1 - v0) * sampler->next();

		float pdf_u = 1.0f / env->width;
		float pdf_v = 1.0f / env->height;

		pdf = pdf_u * pdf_v;

		//return env->sample(u_, v_);
		return Vec3();
	}

	std::vector<float> cdf; // Cumulative distribution function for importance sampling
	float totalLuminance;
	void buildCDF() {
		int numPixels = env->width * env->height;
		cdf.resize(numPixels + 1);
		totalLuminance = 0.0f;

		for (int y = 0; y < env->height; ++y) {
			for (int x = 0; x < env->width; ++x) {
				int idx = y * env->width + x;
				float luminance = env->texels[idx].Lum();

				// Weight by sin(theta) to account for spherical projection
				float theta = M_PI * (y + 0.5f) / env->height;
				float sinTheta = std::sin(theta);
				totalLuminance += luminance * sinTheta;
				cdf[idx + 1] = totalLuminance;
			}
		}
		// Normalize CDF
		for (int i = 1; i <= numPixels; ++i) {
			cdf[i] /= totalLuminance;
		}
	}

	Vec3 sample2(const ShadingData & shadingData, Sampler * sampler, Colour & reflectedColour, float& pdf)
	{
		// Importance sampling based on luminance
		float r = sampler->next(); // Random number [0, 1]
		int idx = std::lower_bound(cdf.begin(), cdf.end(), r) - cdf.begin() - 1;
		int y = idx / env->width;
		int x = idx % env->width;

		// Convert pixel coordinates to spherical coordinates
		float u = (x + sampler->next()) / env->width;  // Add jittering
		float v = (y + sampler->next()) / env->height; // Add jittering
		float theta = v * M_PI;
		float phi = u * 2.0f * M_PI;

		// Convert to Cartesian direction
		float sinTheta = std::sin(theta);
		Vec3 wi(
			sinTheta * std::cos(phi),
			sinTheta * std::sin(phi),
			std::cos(theta)
		);

		// Evaluate radiance and compute PDF
		reflectedColour = evaluate(shadingData, wi);
		float luminance = 0.2126f * reflectedColour.r + 0.7152f * reflectedColour.g + 0.0722f * reflectedColour.b;
		pdf = (luminance * sinTheta) / totalLuminance;

		return wi;
	}

	Colour evaluate2(const ShadingData& shadingData, const Vec3& wi)
	{
		// Convert direction to UV coordinates in latitude-longitude format
		float theta = std::acos(wi.z);
		float phi = std::atan2(wi.y, wi.x);
		if (phi < 0) phi += 2.0f * M_PI;
		float u = phi / (2.0f * M_PI);
		float v = theta / M_PI;

		// Bilinear interpolation (simplified here, could be expanded)
		int x = std::min(static_cast<int>(u * env->width), env->width - 1);
		int y = std::min(static_cast<int>(v * env->height), env->height - 1);
		return env->texels[y * env->width + x];
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		// Samples a point on the bounding sphere of the scene. Feel free to improve this.
		Vec3 p = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		p = p * use<SceneBounds>().sceneRadius;
		p = p + use<SceneBounds>().sceneCentre;
		pdf = 1.0f / (4 * M_PI * SQ(use<SceneBounds>().sceneRadius));
		return p;
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		// Replace this tabulated sampling of environment maps
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		return wi;
	}
};