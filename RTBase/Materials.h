#pragma once

#include "Core.h"
#include "Imaging.h"
#include "Sampling.h"

#include <algorithm>


#pragma warning( disable : 4244)

class BSDF;

class ShadingData
{
public:
	Vec3 x;
	Vec3 wo;
	Vec3 sNormal;
	Vec3 gNormal;
	float tu;
	float tv;
	Frame frame;
	BSDF* bsdf;
	float t;
	ShadingData() {}
	ShadingData(Vec3 _x, Vec3 n)
	{
		x = _x;
		gNormal = n;
		sNormal = n;
		bsdf = NULL;
	}
};

class ShadingHelper
{
public:
	static float fresnelDielectric(float cosTheta, float iorInt, float iorExt)
	{
		// Add code here
		//return 1.0f;
		cosTheta = HelperFunctions::clamp(cosTheta, -1.0f, 1.0f);

		//if light is coming from inside the object
		bool entering = cosTheta > 0.f;
		if (entering)
		{
			std::swap(iorInt, iorExt);
			cosTheta = std::fabs(cosTheta);
		}
		float refractionIndex = iorExt / iorInt;


		float cosThetaI = cosTheta;
		float sinThetaI = sqrtf(std::max(0.0f, 1 - powf(cosThetaI, 2)));

		float sinThetaT = refractionIndex * sinThetaI;
		if (sinThetaT >= 1.f)return 1.f;
		float cosThetaT = sqrtf(std::max(0.0f, 1 - powf(sinThetaT, 2)));

		float FwParallel = (cosThetaI - (refractionIndex * cosThetaT)) / (cosThetaI + (refractionIndex * cosThetaT));
		float FwPerpendicular = ((cosThetaI * refractionIndex) - cosThetaT) / ((cosThetaI *refractionIndex) + cosThetaT);

		return 0.5f * (SQ(FwParallel) + SQ(FwPerpendicular));		
	}
	static Colour fresnelConductor(float cosTheta, Colour ior, Colour k)
	{
		// Add code here
		//return Colour(1.0f, 1.0f, 1.0f);

		//cosTheta = clamp(cosTheta, -1.0f, 1.0f);
		cosTheta = std::fabs(cosTheta);
		float cosSquaredTheta = powf(cosTheta, 2);
		float sinSquaredTheta =  1 - cosSquaredTheta;

		Colour cos2Theta = Colour(cosSquaredTheta, cosSquaredTheta, cosSquaredTheta);
		Colour sin2Theta = Colour(sinSquaredTheta, sinSquaredTheta, sinSquaredTheta);

		Colour iorSquared = ior * ior;
		Colour kSquared = k * k;

		Colour nk2 = iorSquared + kSquared;
		Colour TwoNCos = ior * cosTheta * 2;

		Colour FwParallel = ((nk2 * cosSquaredTheta) - TwoNCos + sin2Theta) / ((nk2 * cosSquaredTheta) + TwoNCos + sin2Theta);
		Colour FwPerpendicular = (nk2  - TwoNCos + cos2Theta) / (nk2 + TwoNCos + cos2Theta);

		return (SQ(FwParallel) + SQ(FwPerpendicular)) * 0.5f;
	}

	//using trowbridge-reitz distribution
	static float lambdaGGX(Vec3 wi, float alpha)
	{
		// Add code here
		//return 1.0f;
		float AbsTanTheta = std::abs(ShadingHelper::TanTheta(wi));
		float alpha2Tans2Theta = (alpha * AbsTanTheta) * (alpha * AbsTanTheta);

		return (-1 + sqrtf(1 + alpha2Tans2Theta)) * 0.5f;
	}
	static float Gggx(Vec3 wi, Vec3 wo, float alpha)
	{
		// Add code here
		//return 1.0f;

		return 1.0f / (1.0f + lambdaGGX(wi, alpha) + lambdaGGX(wo, alpha));
	}
	static float Dggx(Vec3 h, float alpha)
	{
		// Add code here
		//return 1.0f;
		float cosSquaredTheta = ShadingHelper::Cos2Theta(h);
		float alphaSquared = powf(alpha, 2);	
		float denominator =(cosSquaredTheta * (alphaSquared - 1)) + 1;
		return alphaSquared / (denominator * denominator * M_PI);
	}

	static void setIOR(float cosTheta , float& iorI , float& iorT, float intIOR , float extIOR) {
		bool entering = cosTheta > 0;

		iorI = entering ? extIOR : intIOR;
		iorT = entering ? intIOR : extIOR;
		cosTheta = std::fabs(cosTheta);
	}

	static float CosTheta(Vec3 wi)
	{
		return wi.z;
	}

	static float Cos2Theta(Vec3 wi)
	{
		return wi.z * wi.z;
	}

	static float AbsCosTheta(Vec3 wi)
	{
		return std::fabs(wi.z);
	}

	static float Sin2Theta(Vec3 wi)
	{
		return std::max(0.0f, 1 - Cos2Theta(wi));
	}

	static float SinTheta(Vec3 wi)
	{
		return sqrtf(Sin2Theta(wi));
	}

	static float TanTheta(Vec3 wi)
	{
		return SinTheta(wi) / CosTheta(wi);
	}

	static float Tan2Theta(Vec3 wi)
	{
		return Sin2Theta(wi) / Cos2Theta(wi);
	}
	
	static float CosPhi(Vec3 wi)
	{
		float sinTheta = SinTheta(wi);
		return (sinTheta == 0) ? 1.f : HelperFunctions::clamp(wi.x / sinTheta, -1.f, 1.f);
	}
	
	static float SinPhi(Vec3 wi)
	{
		float sinTheta = SinTheta(wi);
		return (sinTheta == 0) ? 0.f : HelperFunctions::clamp(wi.y / sinTheta, -1.f, 1.f);
	}
};

class BSDF
{
public:
	Colour emission;
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf) = 0;
	virtual Colour evaluate(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isPureSpecular() = 0;
	virtual bool isTwoSided() = 0;
	bool isLight()
	{
		return emission.Lum() > 0 ? true : false;
	}
	void addLight(Colour _emission)
	{
		emission = _emission;
	}
	Colour emit(const ShadingData& shadingData, const Vec3& wi)
	{
		return emission;
	}
	virtual float mask(const ShadingData& shadingData) = 0;
};

class DiffuseBSDF : public BSDF
{
public:
	Texture* albedo;
	DiffuseBSDF() = default;
	DiffuseBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		//// Add correct sampling code here
		//Vec3 wi = Vec3(0, 1, 0);
		//pdf = 1.0f;
		//reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		//wi = shadingData.frame.toWorld(wi);
		//return wi;

		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add correct PDF code here
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class MirrorBSDF : public BSDF
{
public:
	Texture* albedo;
	MirrorBSDF() = default;
	MirrorBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Mirror sampling code
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wi = Vec3(-woLocal.x, -woLocal.y, woLocal.z);
		pdf = 1.0f;

		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / wi.z;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Mirror evaluation code
		//return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Mirror PDF
		return 0;
	}
	bool isPureSpecular()
	{
		return true;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};


class ConductorBSDF : public BSDF
{
public:
	Texture* albedo;
	Colour eta;
	Colour k;
	float alpha;
	ConductorBSDF() = default;
	ConductorBSDF(Texture* _albedo, Colour _eta, Colour _k, float roughness)
	{
		albedo = _albedo;
		eta = _eta;
		k = _k;
		alpha = 1.62142f * sqrtf(roughness);
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Conductor sampling code
		//Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		//pdf = wi.z / M_PI;
		//reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		//wi = shadingData.frame.toWorld(wi);
		//return wi;

		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		if (woLocal.z <= 0.0f)
		{
			pdf = 0.0f;
			reflectedColour = Colour(0.0f, 0.0f, 0.0f);
			return Vec3(0.0f, 0.0f, 0.0f);
		}

		if (alpha < 0.001f)
		{
			Vec3 wi = Vec3(-woLocal.x, -woLocal.y, woLocal.z);
			pdf = 1.0f;
			float cosTheta = ShadingHelper::CosTheta(wi);
			Colour F = ShadingHelper::fresnelConductor(cosTheta, eta, k);

			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * F;
			return shadingData.frame.toWorld(wi);
		}


		float r1 = sampler->next();
		float r2 = sampler->next();

		float alphaSquared = powf(alpha, 2);
		float cosTheta = sqrtf((1-r1) / (r1 * ((alphaSquared - 1) + 1)));
		float sinTheta = sqrtf(std::max(0.0f, 1 - powf(cosTheta, 2)));
		float phi = 2 * M_PI * r2;

		Vec3 wmLocal = Vec3(sinTheta * cos(phi), sinTheta * sin(phi), cosTheta);
		Vec3 wiLocal =  ( wmLocal * Dot(woLocal, wmLocal) * 2) - woLocal;

		float D = ShadingHelper::Dggx(wmLocal, alpha);
		float G = ShadingHelper::Gggx(wiLocal, woLocal, alpha);

		Colour F = ShadingHelper::fresnelConductor(Dot(wiLocal, wmLocal), eta, k);

		Colour brdf = F * D * G / (4.0f * woLocal.z * wiLocal.z);
		pdf = D * wmLocal.z / (4 * Dot(woLocal, wmLocal));
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * brdf * fabs(wiLocal.z) / pdf;

		return shadingData.frame.toWorld(wiLocal);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Conductor evaluation code
		//return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;

		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wi);

		if (woLocal.z <= 0.0f || wiLocal.z <= 0.0f)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}

		Vec3 wmLocal = (wiLocal + woLocal).normalize();

		if (wmLocal.z <= 0.0f)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}

		if (alpha < 0.001f)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}

		float D = ShadingHelper::Dggx(wmLocal, alpha);
		float G = ShadingHelper::Gggx(wiLocal, woLocal, alpha);
		Colour F = ShadingHelper::fresnelConductor(Dot(woLocal, wmLocal), eta, k);

		Colour brdf = F * D * G / (4.0f * woLocal.z * wiLocal.z);
		return albedo->sample(shadingData.tu, shadingData.tv) * brdf;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Conductor PDF
		//Vec3 wiLocal = shadingData.frame.toLocal(wi);
		//return SamplingDistributions::cosineHemispherePDF(wiLocal);

		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wi);

		if (woLocal.z <= 0.0f || wiLocal.z <= 0.0f)
		{
			return 0.0f;
		}

		Vec3 wmLocal = (wiLocal + woLocal).normalize();
		if (wmLocal.z <= 0.0f)
		{
			return 0.0f;
		}

		if (alpha < 0.001f)
		{
			return 0.0f;
		}

		float D = ShadingHelper::Dggx(wmLocal, alpha);
		return D * wmLocal.z / (4 * Dot(woLocal, wmLocal));
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class GlassBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	GlassBSDF() = default;
	GlassBSDF(Texture* _albedo, float _intIOR, float _extIOR)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		//Replace this with Glass sampling code
		//Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		//pdf = wi.z / M_PI;
		//reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		//wi = shadingData.frame.toWorld(wi);
		//return wi;

		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		float cosTheta = ShadingHelper::CosTheta(woLocal);

		float iorT, iorI;
		ShadingHelper::setIOR(cosTheta, iorI, iorT, intIOR, extIOR);

		float refractionIndex = iorI / iorT;
		float F = ShadingHelper::fresnelDielectric(cosTheta, iorI, iorT);
		Vec3 wi = Vec3(0, 0, 0);

		bool reflect = sampler->next() < F;

		if (reflect)
		{
			//Reflect
			wi = Vec3(-woLocal.x, -woLocal.y, woLocal.z);
			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv);
			pdf = F;
		}
		else
		{
			//Refract
			float sinThetaI = sqrtf(std::max(0.0f, 1.0f - powf(cosTheta, 2)));
			float sinThetaT = refractionIndex * sinThetaI;

			if (sinThetaT >= 1.0f) {
				//Total internal reflection
				wi = Vec3(-woLocal.x, -woLocal.y, woLocal.z);
				reflectedColour = albedo->sample(shadingData.tu, shadingData.tv);
				pdf = 1.0f;
			}
			else
			{
				float cosThetaT = sqrtf(std::max(0.0f, 1 - powf(sinThetaT, 2)));
				

				wi = Vec3(-woLocal.x * refractionIndex, -woLocal.y * refractionIndex, 0);
				wi.z = cosTheta > 0 ? -cosThetaT : cosThetaT;
				wi = wi.normalize();
				pdf = (1.0f - F);// * (refractionIndex * refractionIndex);
				reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * (pdf * refractionIndex * refractionIndex);
			}
		}


		return shadingData.frame.toWorld(wi.normalize());
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Glass evaluation code
		//return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with GlassPDF
		//Vec3 wiLocal = shadingData.frame.toLocal(wi);
		//return SamplingDistributions::cosineHemispherePDF(wiLocal);
		return 0.f;
	}
	bool isPureSpecular()
	{
		return true;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class DielectricBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	DielectricBSDF() = default;
	DielectricBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Dielectric sampling code
		//Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		//pdf = wi.z / M_PI;
		//reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		//wi = shadingData.frame.toWorld(wi);
		//return wi;

		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		float cosTheta = ShadingHelper::CosTheta(woLocal);
		if (cosTheta <= 0.0f)
		{
			pdf = 0.0f;
			reflectedColour = Colour(0.0f, 0.0f, 0.0f);
			return Vec3(0.0f, 0.0f, 0.0f);
		}

		float iorT, iorI;
		ShadingHelper::setIOR(cosTheta, iorI, iorT, intIOR, extIOR);

		float refractionIndex = iorI / iorT;

		float r1 = sampler->next();
		float r2 = sampler->next();
		float alphaSquared = powf(alpha, 2);

		float cosThetaM = sqrtf((1 - r1) / (r1 * ((alphaSquared - 1) + 1)));
		float sinThetaM = sqrtf(std::max(0.0f, 1 - powf(cosThetaM, 2)));
		float phi = 2 * M_PI * r2;

		Vec3 wmLocal = Vec3(sinThetaM * cos(phi), sinThetaM * sin(phi), cosThetaM);
		Vec3 wiLocal;
		float pdfRefl, pdfRefr;

		float F = ShadingHelper::fresnelDielectric(Dot(wiLocal,wmLocal), iorI, iorT);
		float D = ShadingHelper::Dggx(wmLocal, alpha);

		bool reflect = false;

		if (sampler->next() < F)
		{
			//Reflect
			reflect = true;
			wiLocal = -woLocal + (wmLocal * Dot(woLocal, wmLocal) * 2);
			pdfRefl = D * cosThetaM / (4 * Dot(woLocal, wmLocal));
			pdfRefr = 0;
		}
		else
		{
			//Refract
			float cosThetaI = Dot(woLocal, wmLocal);
			float sinThetaI = sqrtf(std::max(0.0f, 1 - powf(cosThetaI, 2)));
			float sinThetaT = refractionIndex * sinThetaI;

			if (sinThetaT >= 1.0f)
			{
				//Total internal reflection
				reflect = true;
				wiLocal = -woLocal + (wmLocal * Dot(woLocal, wmLocal) * 2);
				pdfRefl = D * cosThetaM / (4 * Dot(woLocal, wmLocal));
				pdfRefr = 0;
				F = 1.0f;
			}
			else
			{
				float cosThetaT = sqrtf(std::max(0.0f, 1 - powf(sinThetaT, 2)));
				
				//H
				wiLocal = -woLocal * refractionIndex + (wmLocal * (refractionIndex * cosThetaI - cosThetaT));
				if (woLocal.z > 0)
					wiLocal = -wiLocal;
				pdfRefl = 0.0f;
				float denominator = refractionIndex * Dot(wiLocal, wmLocal) + Dot(woLocal, wmLocal);
				pdfRefr = (D * Dot(wiLocal, wmLocal)) / powf(denominator, 2);
			}
		}

		pdf = F * pdfRefl + (1 - F) * pdfRefr;
		float G = ShadingHelper::Gggx(wiLocal, woLocal, alpha);

		if (reflect) {
			pdf = D * wmLocal.z / (4 * Dot(woLocal, wmLocal));
			float brdf = F * D * G / (4.0f * woLocal.z * wiLocal.z);
			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * brdf * fabs(wiLocal.z) / pdf;
		}
		else {
			float denominator = refractionIndex * Dot(wiLocal, wmLocal) + Dot(woLocal, wmLocal);
			float bsdf = ((1 - F) * D * G * Dot(wiLocal,wmLocal) * Dot(woLocal,wmLocal) )/ (powf(denominator,2) * wiLocal.z * woLocal.z);
			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * bsdf * fabs(wiLocal.z) / pdf;
		}
		return shadingData.frame.toWorld(wiLocal);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Dielectric evaluation code
		//return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;

		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wi);

		if (woLocal.z <= 0.0f)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}

		float iorT, iorI;
		ShadingHelper::setIOR(woLocal.z, iorI, iorT, intIOR, extIOR);
		float refractionIndex = iorI / iorT;

		Vec3 wmLocal;

		bool reflect = wiLocal.z * woLocal.z > 0;
		if (reflect) {
			wmLocal = (wiLocal + woLocal).normalize();
		}
		else {
			wmLocal = (wiLocal * refractionIndex + woLocal).normalize();
			if (wmLocal.z < 0)
				wmLocal = -wmLocal;
		}

		if (wmLocal.z <= 0.0f)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}

		float F = ShadingHelper::fresnelDielectric(Dot(woLocal, wmLocal), iorI, iorT);
		float D = ShadingHelper::Dggx(wmLocal, alpha);
		float G = ShadingHelper::Gggx(wiLocal, woLocal, alpha);


		float result;
		if (reflect)
			result =  F * D * G / (4.0f * woLocal.z * wiLocal.z);
		else
		{
			float denominator = refractionIndex * Dot(wiLocal, wmLocal) + Dot(woLocal, wmLocal);
			result = (1 - F) * D * G * Dot(wiLocal, wmLocal) * Dot(woLocal, wmLocal) / (powf(denominator, 2) * wiLocal.z * woLocal.z);
		}
		return albedo->sample(shadingData.tu, shadingData.tv) * result;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Dielectric PDF
		//Vec3 wiLocal = shadingData.frame.toLocal(wi);
		//return SamplingDistributions::cosineHemispherePDF(wiLocal);

		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wi);

		if (woLocal.z <= 0.0f)
		{
			return 0.0f;
		}

		float iorT, iorI;
		ShadingHelper::setIOR(woLocal.z, iorI, iorT, intIOR, extIOR);
		float refractionIndex = iorI / iorT;

		Vec3 wmLocal;

		bool reflect = wiLocal.z * woLocal.z > 0;
		if (reflect) {
			wmLocal = (wiLocal + woLocal).normalize();
		}
		else {
			wmLocal = (wiLocal * refractionIndex + woLocal).normalize();
			if (wmLocal.z < 0)
				wmLocal = -wmLocal;
		}

		if (wmLocal.z <= 0.0f)
		{
			return 0.0f;
		}

		float F = ShadingHelper::fresnelDielectric(Dot(woLocal, wmLocal), iorI, iorT);
		float D = ShadingHelper::Dggx(wmLocal, alpha);
		float pdfRefl = D * wmLocal.z / (4 * Dot(woLocal, wmLocal));
		float pdfRefr = 0.0f;

		if (!reflect)
		{
			float denominator = refractionIndex * Dot(wiLocal, wmLocal) + Dot(woLocal, wmLocal);
			pdfRefr = (D * Dot(wiLocal, wmLocal)) / powf(denominator, 2);
		}
		return F * pdfRefl + (1 - F) * pdfRefr;
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return false;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class OrenNayarBSDF : public BSDF
{
public:
	Texture* albedo;
	float sigma;
	OrenNayarBSDF() = default;
	OrenNayarBSDF(Texture* _albedo, float _sigma)
	{
		albedo = _albedo;
		sigma = _sigma;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with OrenNayar sampling code
		//Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		//pdf = wi.z / M_PI;
		//reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		//wi = shadingData.frame.toWorld(wi);
		//return wi;

		Vec3 wiLocal = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wiLocal.z / M_PI;
		Vec3 wi = shadingData.frame.toWorld(wiLocal);
		reflectedColour = evaluate(shadingData, wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with OrenNayar evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;

		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wi);

		if (woLocal.z <= 0.0f || wiLocal.z <= 0.0f)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}

		float sigmaSquared = powf(sigma, 2);
		float A = 1 - (sigmaSquared / (2 * (sigmaSquared + 0.33f)));
		float B = 0.45f * sigmaSquared / (sigmaSquared + 0.09f);

		float sinThetaI = ShadingHelper::SinTheta(wiLocal);
		float sinThetaO = ShadingHelper::SinTheta(woLocal);

		float maxCos = 0;

		if (sinThetaI > 1e-4 && sinThetaO > 1e-4) {
			float sinPhiI = ShadingHelper::SinPhi(wiLocal);
			float sinPhiO = ShadingHelper::SinPhi(woLocal);
			float cosPhiI = ShadingHelper::CosPhi(wiLocal);
			float cosPhiO = ShadingHelper::CosPhi(woLocal);
			
			float dCos = cosPhiI * cosPhiO + sinPhiI * sinPhiO;
			maxCos = std::max(0.0f, dCos);
		}

		float sinAlpha, tanBeta;
		if (ShadingHelper::AbsCosTheta(wiLocal) > ShadingHelper::AbsCosTheta(woLocal))
		{
			sinAlpha = sinThetaO;
			tanBeta = sinThetaI / ShadingHelper::AbsCosTheta(wiLocal);
		}
		else
		{
			sinAlpha = sinThetaI;
			tanBeta = sinThetaO / ShadingHelper::AbsCosTheta(woLocal);
		}

		float orenNayar = A + B * maxCos * sinAlpha * tanBeta;

		return albedo->sample(shadingData.tu, shadingData.tv) * orenNayar / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with OrenNayar PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class PlasticBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	PlasticBSDF() = default;
	PlasticBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}
	float alphaToPhongExponent()
	{
		return (2.0f / SQ(std::max(alpha, 0.001f))) - 2.0f;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Plastic sampling code
		//Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		//pdf = wi.z / M_PI;
		//reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		//wi = shadingData.frame.toWorld(wi);
		//return wi;


		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		float cosTheta = ShadingHelper::CosTheta(woLocal);

		if (cosTheta <= 0.0f)
		{
			pdf = 1.0f;
			reflectedColour = Colour(0.0f, 0.0f, 0.0f);
			return Vec3(0.0f, 0.0f, 0.0f);
		}

		float iorT, iorI;
		ShadingHelper::setIOR(cosTheta, iorI, iorT, intIOR, extIOR);

		float refractionIndex = iorI / iorT;
		float ks = ShadingHelper::fresnelDielectric(cosTheta, iorI, iorT);
		float kd = 1.0f - ks;
		Vec3 wiLocal;
		float pdfDiffuse, pdfGlossy;

		float e = alphaToPhongExponent();

		float s1 = sampler->next();
		float s2 = sampler->next();

		Vec3 wrLocal = Vec3(-woLocal.x, -woLocal.y, woLocal.z);
		if (sampler->next() > ks)
		{
			//Diffuse
			wiLocal = SamplingDistributions::cosineSampleHemisphere(s1, s2);
			pdfDiffuse = wiLocal.z / M_PI;
			pdfGlossy = 0;;
		}
		else
		{
			//Specular
			float cosTheta = powf(s1, 1.0f / (e + 1.f));
			float sinTheta = sqrtf(std::max(0.0f, 1.0f - powf(cosTheta, 2)));
			float phi = 2 * M_PI * s2;

			Vec3 lobeDir = Vec3(sinTheta * cos(phi), sinTheta * sin(phi), cosTheta);

			Frame wrFrame;
			wrFrame.fromVector(wrLocal);
			wiLocal = wrFrame.toLocal(lobeDir);

			float maxWrW = std::max(0.0f, Dot(wrLocal, wiLocal));
			pdfGlossy = ((e + 1) * powf(maxWrW, e) / (2 * M_PI));
			pdfDiffuse = 0;
		}
		float cosAlpha = std::max(0.0f, Dot(wrLocal, wiLocal));
		pdf = (kd * (wiLocal.z / M_PI)) + (ks * (e+1) * powf(cosAlpha, e) / (2 * M_PI));


		Colour diffuse = albedo->sample(shadingData.tu, shadingData.tv) * kd / M_PI;
		float gloss = ks * ((e + 2) / (2 * M_PI)) * powf(cosAlpha, e);
		Colour glossy = Colour(gloss, gloss, gloss);
		reflectedColour = (diffuse + glossy);// *fabs(wiLocal.z) / std::max(pdf, 1e-6f);

		return shadingData.frame.toWorld(wiLocal);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Plastic evaluation code
		//return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;

		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wi);

		if (wiLocal.z <= 0.0f || woLocal.z <= 0.0f)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}

		float cosTheta = woLocal.z;
		float iorT, iorI;
		ShadingHelper::setIOR(cosTheta, iorI, iorT, intIOR, extIOR);

		float refractionIndex = iorI / iorT;
		float ks = ShadingHelper::fresnelDielectric(cosTheta, iorI, iorT);
		float kd = 1.0f - ks;
		float e = alphaToPhongExponent();


		Colour diffuse = albedo->sample(shadingData.tu, shadingData.tv) * kd / M_PI;
		Vec3 wrLocal = Vec3(-woLocal.x, -woLocal.y, woLocal.z);
		float cosAlpha = std::max(0.0f, Dot(wrLocal, wiLocal));
		float gloss = ks * ((e + 2) / (2 * M_PI)) * powf(cosAlpha, e);
		Colour glossy = Colour(gloss, gloss, gloss);

		return (diffuse + glossy);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Plastic PDF
		//Vec3 wiLocal = shadingData.frame.toLocal(wi);
		//return SamplingDistributions::cosineHemispherePDF(wiLocal);

		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wi);

		if (wiLocal.z <= 0.0f || woLocal.z <= 0.0f)
		{
			return 0.0f;
		}

		float cosTheta = woLocal.z;
		float iorT, iorI;

		ShadingHelper::setIOR(cosTheta, iorI, iorT, intIOR, extIOR);

		float refractionIndex = iorI / iorT;
		float ks = ShadingHelper::fresnelDielectric(cosTheta, iorI, iorT);
		float kd = 1.0f - ks;
		float e = alphaToPhongExponent();

		float pdfDiffuse = wiLocal.z / M_PI;

		Vec3 wrLocal = Vec3(-woLocal.x, -woLocal.y, woLocal.z);
		float cosAlpha = std::max(0.0f, Dot(wrLocal, wiLocal));
		float pdfGlossy = ((e + 1) * powf(cosAlpha, e) / (2 * M_PI));

		return (kd * pdfDiffuse) + (ks * pdfGlossy);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class LayeredBSDF : public BSDF
{
public:
	BSDF* base;
	Colour sigmaa;
	float thickness;
	float intIOR;
	float extIOR;
	LayeredBSDF() = default;
	LayeredBSDF(BSDF* _base, Colour _sigmaa, float _thickness, float _intIOR, float _extIOR)
	{
		base = _base;
		sigmaa = _sigmaa;
		thickness = _thickness;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Add code to include layered sampling
		return base->sample(shadingData, sampler, reflectedColour, pdf);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add code for evaluation of layer
		return base->evaluate(shadingData, wi);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add code to include PDF for sampling layered BSDF
		return base->PDF(shadingData, wi);
	}
	bool isPureSpecular()
	{
		return base->isPureSpecular();
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return base->mask(shadingData);
	}
};