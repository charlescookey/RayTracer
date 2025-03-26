#pragma once

#include "Core.h"
#include "Sampling.h"
#include "Geometry.h"
#include "Imaging.h"
#include "Materials.h"
#include "Lights.h"
#include "Scene.h"
#include "GamesEngineeringBase.h"
#include <thread>
#include <functional>

#define tileSize 16
#define threadNum 12
#define MAX_DEPTH 4


class RayTracer
{
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom* samplers;
	std::thread** threads;
	int numProcs;
	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas)
	{
		scene = _scene;
		canvas = _canvas;
		film = new Film();
		film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new BoxFilter());
		SYSTEM_INFO sysInfo;
		GetSystemInfo(&sysInfo);
		numProcs = sysInfo.dwNumberOfProcessors;
		threads = new std::thread * [numProcs];
		samplers = new MTRandom[numProcs];
		clear();
	}
	void clear()
	{
		film->clear();
	}
	Colour computeDirect2(ShadingData shadingData, Sampler* sampler)
	{
		// Is surface is specular we cannot computing direct lighting
		if (shadingData.bsdf->isPureSpecular() == true)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		// Compute direct lighting here
		return Colour(0.0f, 0.0f, 0.0f);

		float pmf;
		Light* sL = scene->sampleLight(sampler, pmf);

		Colour cL;
		float pdf;
		Vec3 XL;
		Vec3 NL;
		
		Vec3 X;
		Vec3 N;

		if (sL->isArea()) {
			XL = sL->sample(shadingData, sampler, cL, pdf);
			NL = sL->normal(shadingData, XL);


			X = shadingData.x;
			N = shadingData.gNormal;

			Vec3 wi = XL - X;

			float r = wi.length();
			float cosTheta = Dot(wi, N);
			float cosThetaPrime = -Dot(wi, NL);

			float Geom = (cosTheta * cosThetaPrime) / (r * r);


			//auto GeometryTerm  = (Dot(wi , ))
		}


	}

	Colour computeDirect(ShadingData shadingData, Sampler* sampler)
	{
		if (shadingData.bsdf->isPureSpecular() == true)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		// Sample a light
		float pmf;
		Light* light = scene->sampleLight(sampler, pmf);
		// Sample a point on the light
		float pdf;
		Colour emitted;
		Vec3 p = light->sample(shadingData, sampler, emitted, pdf);
		if (light->isArea())
		{
			// Calculate GTerm
			Vec3 wi = p - shadingData.x;
			float l = wi.lengthSq();
			wi = wi.normalize();
			float GTerm = (max(Dot(wi, shadingData.sNormal), 0.0f) * max(-Dot(wi, light->normal(shadingData, wi)), 0.0f)) / l;
			if (GTerm > 0)
			{
				// Trace
				if (scene->visible(shadingData.x, p))
				{
					// Shade
					return shadingData.bsdf->evaluate(shadingData, wi) * emitted * GTerm / (pmf * pdf);
				}
			}
		}
		else
		{
			// Calculate GTerm
			Vec3 wi = p;
			float GTerm = max(Dot(wi, shadingData.sNormal), 0.0f);
			if (GTerm > 0)
			{
				// Trace
				if (scene->visible(shadingData.x, shadingData.x + (p * 10000.0f)))
				{
					// Shade
					return shadingData.bsdf->evaluate(shadingData, wi) * emitted * GTerm / (pmf * pdf);
				}
			}
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	Colour pathTrace(float px, float py, Sampler* sampler) {
		Colour pathThroughput(1.0f, 1.0f, 1.0f);
		Ray ray = scene->camera.generateRay(px + sampler->next(), py+sampler->next());
		return pathTrace(ray, pathThroughput, 0, samplers, true);
	}

	Colour pathTrace(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler, bool canHitLight = true)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				if (canHitLight == true)
				{
					return pathThroughput * shadingData.bsdf->emit(shadingData, shadingData.wo);
				}
				else
				{
					return Colour(0.0f, 0.0f, 0.0f);
				}
			}
			Colour direct = pathThroughput * computeDirect(shadingData, sampler);
			if (depth > MAX_DEPTH)
			{
				return direct;
			}
			float russianRouletteProbability = min(pathThroughput.Lum(), 0.9f);
			if (sampler->next() < russianRouletteProbability)
			{
				pathThroughput = pathThroughput / russianRouletteProbability;
			}
			else
			{
				return direct;
			}
			Colour indirect;
			float pdf;
			//Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
			Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, indirect, pdf);
			pathThroughput = pathThroughput * indirect * fabsf(Dot(wi, shadingData.sNormal)) / pdf;
			r.init(shadingData.x + (wi * EPSILON), wi);
			return (direct + pathTrace(r, pathThroughput, depth + 1, sampler, shadingData.bsdf->isPureSpecular()));
		}
		return scene->background->evaluate(shadingData, r.dir);
	}
	Colour direct(Ray& r, Sampler* sampler)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return computeDirect(shadingData, sampler);
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}

	Colour albedo(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return shadingData.bsdf->evaluate(shadingData, Vec3(0, 1, 0));
		}
		return scene->background->evaluate(shadingData, r.dir);
	}
	Colour viewNormals(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t < FLT_MAX)
		{
			ShadingData shadingData = scene->calculateShadingData(intersection, r);
			return Colour(fabsf(shadingData.sNormal.x), fabsf(shadingData.sNormal.y), fabsf(shadingData.sNormal.z));
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	void render2()
	{
		film->incrementSPP();
		for (unsigned int y = 0; y < film->height; y++)
		{
			for (unsigned int x = 0; x < film->width; x++)
			{
				float px = x + 0.5f;
				float py = y + 0.5f;
				Ray ray = scene->camera.generateRay(px, py);
				//Colour col = viewNormals(ray);
				Colour col = albedo(ray);
				film->splat(px, py, col);
				unsigned char r = (unsigned char)(col.r * 255);
				unsigned char g = (unsigned char)(col.g * 255);
				unsigned char b = (unsigned char)(col.b * 255);
				canvas->draw(x, y, r, g, b);
			}
		}
	}

	void render()
	{
		film->incrementSPP();

		int tilesX = (film->width + tileSize - 1) / tileSize;
		int tilesY = (film->height + tileSize - 1) / tileSize;

		int totalTiles = tilesX * tilesY;

		std::atomic<int> tileNum(0);
		std::vector<std::thread> threads;

		auto threadFunc = [&]() {
			unsigned long i;
			while ((i = tileNum.fetch_add(1)) < totalTiles) {
				int tileX = i / tilesX;
				int tileY = i % tilesX;

				renderTile(tileX, tileY, film->width, film->height, scene, canvas);

			}
			};

		for (int i = 0; i < threadNum; i++) {
			threads.emplace_back(threadFunc);
		}

		for (auto& thread : threads) {
			thread.join();
		}

	}
	void renderTile(int tileX, int tileY, int sizeX, int sizeY, Scene* scene, GamesEngineeringBase::Window* canvas)
	{
		int startX = tileX * tileSize;
		int startY = tileY * tileSize;
		int endX = min(startX + tileSize, sizeX);
		int endY = min(startY + tileSize, sizeY);

		for (unsigned int y = startY; y < endY; y++)
		{
			for (unsigned int x = startX; x < endX; x++)
			{
				float px = x + 0.5f;
				float py = y + 0.5f;
				Ray ray = scene->camera.generateRay(px, py);
				Colour col = viewNormals(ray);
				//Colour col = albedo(ray);
				//Colour col = direct(ray, samplers);
				//Colour col = pathTrace(px,py, samplers);
				film->splat(px, py, col);
				unsigned char r = (unsigned char)(col.r * 255);
				unsigned char g = (unsigned char)(col.g * 255);
				unsigned char b = (unsigned char)(col.b * 255);
				//unsigned char r;
				//unsigned char g;
				//unsigned char b;
				film->tonemap(x, y, r, g, b);
				canvas->draw(x, y, r, g, b);
			}
		}

		//canvas->present();
	}
	int getSPP()
	{
		return film->SPP;
	}
	void saveHDR(std::string filename)
	{
		film->save(filename);
	}
	void savePNG(std::string filename)
	{
		stbi_write_png(filename.c_str(), canvas->getWidth(), canvas->getHeight(), 3, canvas->getBackBuffer(), canvas->getWidth() * 3);
	}



	void connectToCamera(Vec3 p, Vec3 n, Colour col) {
		float x;
		float y;
		if (!scene->camera.projectOntoCamera(p, x, y)) {
			return;
		}

		Vec3 cameraDir = scene->camera.origin - p;
		cameraDir = cameraDir.normalize();


		float costheta = Dot(scene->camera.viewDirection, cameraDir);

		if (costheta <= 0) {
			return;
		}


		float we = 1.f / (scene->camera.Afilm * std::pow(costheta, 4));

		Colour contrib = col * we;
		
		film->splat(x, y, contrib);
	};

	void lightTrace(Sampler* sampler) {
		float pmf;
		float pdfPosition;
		float pdfDirection;
		Light* light = scene->sampleLight(sampler, pmf);


		if (light->isArea())
		{
			Vec3 p = light->samplePositionFromLight(sampler, pdfPosition);
			Vec3 wi = light->sampleDirectionFromLight(sampler, pdfDirection);


			ShadingData shadingData;

			Colour Le = light->evaluate(shadingData, -wi) / pdfPosition;

			connectToCamera(p, wi, Le);



			Ray ray;
			ray.init(p, wi);

			Colour pathThroughput(1.0f, 1.0f, 1.0f);///CORRECT THIS LATER

			lightTracePath(ray, pathThroughput, Le, sampler);
			
		}
	};
	void lightTracePath(Ray& r, Colour pathThroughput, Colour Le, Sampler* sampler) {
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);

		if (shadingData.t < FLT_MAX)
		{
			Vec3 wi = scene->camera.origin - shadingData.x;
			wi = wi.normalize();
			Colour col = pathThroughput * shadingData.bsdf->evaluate(shadingData, wi) * Le;

			connectToCamera(shadingData.x, wi, col);

			if (shadingData.bsdf->isLight())
			{
				return;
			}
			int depth = 0; ///CORRECT THIS LATER
			if (depth > MAX_DEPTH)
			{
				return ;
			}


			float russianRouletteProbability = min(pathThroughput.Lum(), 0.9f);
			if (sampler->next() < russianRouletteProbability)
			{
				pathThroughput = pathThroughput / russianRouletteProbability;
			}
			else
			{
				return ;
			}

			Colour bsdf;
			float pdf;
			Vec3 newWi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
			pdf = SamplingDistributions::cosineHemispherePDF(wi);
			newWi = shadingData.frame.toWorld(newWi);


			bsdf = shadingData.bsdf->evaluate(shadingData, newWi);
			pathThroughput = pathThroughput * bsdf * fabsf(Dot(newWi, shadingData.sNormal)) / pdf;
			
			
			Ray newRay;
			newRay.init(shadingData.x + (newWi * EPSILON), newWi);
			lightTracePath(newRay, pathThroughput, Le, sampler);
		}
	};
};