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
	virtual Colour evaluate(const Vec3& wi) = 0;
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
	Colour evaluate(const Vec3& wi)
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
		//Vec3 wi = Vec3(0, 0, 1);
		//pdf = 1.0f;
		//Frame frame;
		//frame.fromVector(triangle->gNormal());
		//return frame.toWorld(wi);

		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::cosineHemispherePDF(wi);
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
	Colour evaluate(const Vec3& wi)
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
	std::vector<float> luminance;
	int nu, nv; 
	EnvironmentMap(Texture* _env)
	{
		env = _env;
		nu = env->width;
		nv = env->height;
		luminance.resize(env->width * env->height);
		build();
	}

	Vec3 sample2(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		 //Assignment: Update this code to importance sampling lighting based on luminance of each pixel
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		reflectedColour = evaluate(wi);
		return wi;

	}
	Colour evaluate(const Vec3& wi)
	{
		float u = atan2f(wi.z, wi.x);
		u = (u < 0.0f) ? u + (2.0f * M_PI) : u;
		u = u / (2.0f * M_PI);
		float v = acosf(wi.y) / M_PI;
		return env->sample(u, v);
	}
	float PDF2(const ShadingData& shadingData, const Vec3& wi)
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
	float funcInt;

	void build() {
		std::vector<float> func(nu * nv);
		funcInt = 0.0f;
		for (int v = 0; v < nv; ++v) {
			float theta = (v + 0.5f) / nv * M_PI;
			float sinTheta = sinf(theta);
			for (int u = 0; u < nu; ++u) {
				int idx = v * nu + u;
				float lum = env->texels[idx].Lum();
				func[idx] = lum * sinTheta;
				funcInt += func[idx];
			}
		}


		conditional_cdf.resize(nv);
		for (int v = 0; v < nv; ++v) {
			conditional_cdf[v].resize(nu);
			float sum = 0.0f;
			for (int u = 0; u < nu; ++u) {
				conditional_cdf[v][u] = func[v * nu + u];
				sum += conditional_cdf[v][u];
			}
			
			if (sum > 0.0f) {
				for (int u = 0; u < nu; ++u) {
					conditional_cdf[v][u] /= sum;
				}
			}
		}

		
		marginal_cdf.resize(nv);
		for (int v = 0; v < nv; ++v) {
			float sum = 0.0f;
			for (int u = 0; u < nu; ++u) {
				sum += func[v * nu + u];
			}
			marginal_cdf[v] = sum;
		}
		
		if (funcInt > 0.0f) {
			for (int v = 0; v < nv; ++v) {
				marginal_cdf[v] /= funcInt;
			}
		}
	}


	void buildDistributions() {
		if (!env || !env->texels) return;

		int width = env->width;
		int height = env->height;
		funcInt = 0;

		luminance_cdf.resize(width * height);
		marginal_cdf.resize(height);
		conditional_cdf.resize(height, std::vector<float>(width));


		std::vector<float> table(width * height);
		for (int v = 0; v < height; v++) {
			float st = sinf(((float)v + 0.5f / (float)height) * M_PI);
			for (int u = 0; u < width; u++) {
				int idx = v * width + u;
				table[idx] = env->texels[idx].Lum() * st;
				funcInt += table[idx];
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
	}

	float Sample1D(std::vector<float>& dist, float u, float* pdf, int* offset)
	{
		float cdf = 0.0f;
		int index = 0;
		for (int i = 0; i < dist.size(); i++)
		{
			cdf += dist[i];
			if (u <= cdf)
			{
				index = i;
				break;
			}
		}
		index = std::min(index, (int)dist.size() - 1);
		if (offset) *offset = index;
		if (pdf) *pdf = dist[index] / dist.size();

		float t = (u - (cdf - dist[index])) / dist[index];
		return (index + t) / (float)dist.size();
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
	
	Vec3 sampleDirectionFromLight2(Sampler* sampler, float& pdf)
	{
		// Replace this tabulated sampling of environment maps
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		return wi;


	}

	Vec2 SampleContinuous(float u0, float u1, float* pdf)  {
		float pdfs[2];
		int v;
		// Sample v from the marginal distribution
		float d1 = Sample1D(marginal_cdf, u1, &pdfs[1], &v);
		// Sample u from the conditional distribution
		float d0 = Sample1D(conditional_cdf[v], u0, &pdfs[0], nullptr);
		*pdf = pdfs[0] * pdfs[1];
		return Vec2(d0, d1);
	}

	// Compute the PDF at a given (u, v)
	float PdfUV(const Vec2& p) const {
		int iu = HelperFunctions::clamp(static_cast<int>(p[0] * nu), 0, nu - 1);
		int iv = HelperFunctions::clamp(static_cast<int>(p[1] * nv), 0, nv - 1);
		return (conditional_cdf[iv][iu] * nu * nv) / funcInt;
	}

	// Convert (u, v) to a direction vector
	Vec3 UVToDirection(float u, float v) const {
		float theta = v * M_PI;
		float phi = u * 2.0f * M_PI;
		float sinTheta = sinf(theta);
		float cosTheta = cosf(theta);
		float sinPhi = sinf(phi);
		float cosPhi = cosf(phi);
		return Vec3(sinTheta * cosPhi, cosTheta, sinTheta * sinPhi);
	}

	// Convert direction to (u, v)
	Vec2 DirectionToUV(const Vec3& wi) const {
		float u = atan2f(wi.z, wi.x);
		u = (u < 0.0f) ? u + (2.0f * M_PI) : u;
		u = u / (2.0f * M_PI);
		float v = acosf(wi.y) / M_PI;
		return Vec2(u, v);
	}

	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Sample (u, v) using the 2D distribution
		float u0 = sampler->next();
		float u1 = sampler->next();
		Vec2 uv = SampleContinuous(u0, u1, &pdf);
		// Convert (u, v) to a direction
		Vec3 wi = UVToDirection(uv[0], uv[1]);
		// Adjust the PDF for the change of variables to spherical coordinates
		float theta = uv[1] * M_PI;
		float sinTheta = sinf(theta);
		pdf *= (2.0f * M_PI * M_PI) / sinTheta; // Jacobian factor
		reflectedColour = evaluate(wi);
		return wi;
	}


	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		Vec2 uv = DirectionToUV(wi);
		float pdfUV = PdfUV(uv);
		float theta = uv[1] * M_PI;
		float sinTheta = sinf(theta);
		return pdfUV * (2.0f * M_PI * M_PI) / sinTheta;
	}

	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		// Sample (u, v) using the 2D distribution
		float u0 = sampler->next();
		float u1 = sampler->next();
		Vec2 uv = SampleContinuous(u0, u1, &pdf);
		// Convert (u, v) to a direction
		Vec3 wi = UVToDirection(uv[0], uv[1]);
		// Adjust the PDF for the change of variables to spherical coordinates
		float theta = uv[1] * M_PI;
		float sinTheta = sinf(theta);
		pdf *= (2.0f * M_PI * M_PI) / sinTheta;
		return wi;
	}
};

