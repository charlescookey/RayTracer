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

#include "OpenImageDenoise/oidn.hpp"

#define tileSize 16
#define threadNum 12
#define MAX_DEPTH 4


struct VPL {
	ShadingData shadingData;
	Colour Le;

	VPL() : Le(0.0f, 0.0f, 0.0f) {}
	VPL(ShadingData _shadingData, Colour _Le) : shadingData(_shadingData), Le(_Le) {}
};

class RayTracer
{
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom* samplers;
	std::thread** threads;
	int numProcs;


	//Denoiser
	oidn::DeviceRef device;

	//Buffers
	oidn::BufferRef colourBuf;
	oidn::BufferRef albedoBuf;
	oidn::BufferRef normalBuf;

	//Filter
	oidn::FilterRef filter;

	Film* albedoFilm;
	Film* normalFilm;


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

		traceVPLs(samplers, 50);



		//denoiser
		device = oidn::newDevice();
		//device = oidn::newDevice(oidn::DeviceType::CPU);
		device.commit();

		const char* errorMessage;
		if (device.getError(errorMessage) != oidn::Error::None)
			std::cout << "Error: " << errorMessage << std::endl;

		colourBuf = device.newBuffer(film->width * film->height * 3 * sizeof(float));
		albedoBuf = device.newBuffer(film->width * film->height * 3 * sizeof(float));
		normalBuf = device.newBuffer(film->width * film->height * 3 * sizeof(float));

		//Expensive operatoiom, add print message 
		filter = device.newFilter("RT");

		filter.setImage("color", colourBuf, oidn::Format::Float3, film->width, film->height); // beauty
		filter.setImage("albedo", albedoBuf, oidn::Format::Float3, film->width, film->height); // auxiliary
		filter.setImage("normal", normalBuf, oidn::Format::Float3, film->width, film->height); // auxiliary
		filter.setImage("output", colourBuf, oidn::Format::Float3, film->width, film->height); // denoised beauty
		filter.set("hdr", true); // beauty image is HDR
		filter.commit();

		albedoFilm = new Film();
		albedoFilm->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new BoxFilter());

		normalFilm = new Film();
		normalFilm->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new BoxFilter());

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
		return pathTrace(ray, pathThroughput, 0, sampler, true);
	}
	
	Colour pathTraceRad(float px, float py, Sampler* sampler) {
		Colour pathThroughput(1.0f, 1.0f, 1.0f);
		Ray ray = scene->camera.generateRay(px + sampler->next(), py+sampler->next());
		return pathTraceRadiosity(ray, pathThroughput, 0, sampler, true);
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
		return scene->background->evaluate( r.dir);
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
		return scene->background->evaluate(r.dir);
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
	
	void renderST()
	{
		film->incrementSPP();
		for (unsigned int y = 0; y < film->height; y++)
		{
			for (unsigned int x = 0; x < film->width; x++)
			{
				float px = x + 0.5f;
				float py = y + 0.5f;
				Ray ray = scene->camera.generateRay(px, py);
				
				//Colour col = albedo(ray);
				//film->splat(px, py, col);

				Colour col = pathTrace(px, py, samplers);
				film->splat(px, py, col);

				unsigned char r;
				unsigned char g;
				unsigned char b;
				film->tonemap(x, y, r, g, b);

				canvas->draw(x, y, r, g, b);
			}
		}
	}

	void renderSTLight()
	{
		film->incrementSPP();

		for (unsigned int y = 0; y < film->height; y++)
		{
			for (unsigned int x = 0; x < film->width; x++)
			{
				lightTrace(samplers);
			}
		}

		for (unsigned int y = 0; y < film->height; y++)
		{
			for (unsigned int x = 0; x < film->width; x++)
			{
				unsigned char r;
				unsigned char g;
				unsigned char b;
				film->tonemap(x, y, r, g, b);

				canvas->draw(x, y, r, g, b);
			}
		}
	}

	void renderSTMLT()
	{
		film->incrementSPP();

		PSSMLTRender(100000);

		for (unsigned int y = 0; y < film->height; y++)
		{
			for (unsigned int x = 0; x < film->width; x++)
			{
				unsigned char r;
				unsigned char g;
				unsigned char b;
				film->tonemap(x, y, r, g, b);
				canvas->draw(x, y, r, g, b);
			}
		}
	}

	int type = 2;
	
	void render(){
		if (type == 0){
			renderMT();
		}
		else if (type == 1) {
			renderSTLight();//light tracing
		}
		else if (type == 2){
			renderSTMLT();//Priamary Sample Space Metropolis Light Transport
		}
		else if (type == 3){
			renderMTAndDenoise();//MT PathTrace + Denoise
		}
		else
		{
			renderST();

		}
	}

	void renderMT()
	{
		film->incrementSPP();

		//there is a bug here, inn canvas draw,int index = ((y * width) + x) * 3; so we use y as width

		int tilesY = (film->width + tileSize - 1) / tileSize;
		int tilesX = (film->height + tileSize - 1) / tileSize;

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

	void renderMTAndDenoise()
	{
		film->incrementSPP();

		//there is a bug here, inn canvas draw,int index = ((y * width) + x) * 3; so we use y as width

		int tilesY = (film->width + tileSize - 1) / tileSize;
		int tilesX = (film->height + tileSize - 1) / tileSize;

		int totalTiles = tilesX * tilesY;

		std::atomic<int> tileNum(0);
		std::vector<std::thread> threads;

		auto threadFunc = [&]() {
			unsigned long i;
			while ((i = tileNum.fetch_add(1)) < totalTiles) {
				int tileX = i / tilesX;
				int tileY = i % tilesX;

				getTileFilms(tileX, tileY, film->width, film->height, scene, canvas);
			}
			};

		for (int i = 0; i < threadNum; i++) {
			threads.emplace_back(threadFunc);
		}

		for (auto& thread : threads) {
			thread.join();
		}

		//Denoise
		denoise();


		for (unsigned int y = 0; y < film->height; y++)
		{
			for (unsigned int x = 0; x < film->width; x++)
			{
				unsigned char r;
				unsigned char g;
				unsigned char b;
				film->tonemap(x, y, r, g, b);
				canvas->draw(x, y, r, g, b);
			}
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
				//Colour col = viewNormals(ray);
				//Colour col = albedo(ray);
				//Colour col = direct(ray, samplers);


				//Colour col = pathTrace(px,py, samplers);
				Colour col = pathTraceRad(px, py, samplers);

				film->splat(px, py, col);
				//unsigned char r = (unsigned char)(col.r * 255);
				//unsigned char g = (unsigned char)(col.g * 255);
				//unsigned char b = (unsigned char)(col.b * 255);
				unsigned char r;
				unsigned char g;
				unsigned char b;
				film->tonemap(x, y, r, g, b);
				canvas->draw(x, y, r, g, b);
			}
		}

		//canvas->present();
	}

	void getTileFilms(int tileX, int tileY, int sizeX, int sizeY, Scene* scene, GamesEngineeringBase::Window* canvas)
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
				normalFilm->splat(px, py, col);

				col = albedo(ray);
				albedoFilm->splat(px, py, col);

				col = pathTrace(px, py, samplers);
				film->splat(px, py, col);
			}
		}
	}


	void denoise()
	{
		int numPixels = film->width * film->height;
		// Copy data to buffers

		float* colourData = (float*)normalBuf.getData();

		for (int i = 0; i < numPixels; i++)
		{
			colourData[i * 3 + 0] = normalFilm->film[i].r;
			colourData[i * 3 + 1] = normalFilm->film[i].g;
			colourData[i * 3 + 2] = normalFilm->film[i].b;
		}

		colourData = (float*)albedoBuf.getData();

		for (int i = 0; i < numPixels; i++)
		{
			colourData[i * 3 + 0] = albedoFilm->film[i].r;
			colourData[i * 3 + 1] = albedoFilm->film[i].g;
			colourData[i * 3 + 2] = albedoFilm->film[i].b;
		}

		colourData = (float*)colourBuf.getData();

		for (int i = 0; i < numPixels; i++)
		{
			colourData[i * 3 + 0] = film->film[i].r;
			colourData[i * 3 + 1] = film->film[i].g;
			colourData[i * 3 + 2] = film->film[i].b;
		}

		// Run the filter
		filter.execute();

		// Copy the denoised data back to the film

		for (int i = 0; i < numPixels; i++)
		{
			film->film[i].r = colourData[i * 3 + 0];
			film->film[i].g = colourData[i * 3 + 1];
			film->film[i].b = colourData[i * 3 + 2];
		}

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
		if (!scene->camera.projectOntoCamera(p, x, y) || !scene->visible(p , scene->camera.origin)) {
			return;
		}

		Vec3 cameraDir = p - scene->camera.origin;// -p;
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


		//if (light->isArea())
		{
			Vec3 p = light->samplePositionFromLight(sampler, pdfPosition);
			Vec3 wi = light->sampleDirectionFromLight(sampler, pdfDirection);

			Colour Le = light->evaluate(- wi) / pdfPosition;

			connectToCamera(p, wi, Le);

			Ray ray;
			ray.init(p, wi);

			Colour pathThroughput(1.0f, 1.0f, 1.0f);///CORRECT THIS LATER

			lightTracePath(ray, pathThroughput, Le, sampler, 0);
			
		}
	};
	
	void lightTracePath(Ray& r, Colour pathThroughput, Colour Le, Sampler* sampler, int depth) {
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
			//int depth = 0; ///CORRECT THIS LATER
			if (depth > MAX_DEPTH)
			{
				return;
			}

			float russianRouletteProbability = min(pathThroughput.Lum(), 0.9f);
			if (sampler->next() < russianRouletteProbability)
			{
				pathThroughput = pathThroughput / russianRouletteProbability;
			}
			else
			{
				return;
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
			lightTracePath(newRay, pathThroughput, Le, sampler, depth+1);
		}
	};

	std::vector<VPL> VPLs;

	void traceVPLs(Sampler* sampler, int N_VPLs) {
		VPLs.clear();
		

		for (int i = 0; i < N_VPLs; i++) {

			float pmf;
			Light* light = scene->sampleLight(sampler, pmf);
			
			float pdfPosition;
			float pdfDirection;
			Vec3 p = light->samplePositionFromLight(sampler, pdfPosition);
			Vec3 wi = light->sampleDirectionFromLight(sampler, pdfDirection);
			//float pmf = 1 / N_VPLs;


			Vec3 lightNormal = light->normal(ShadingData(), p);

			/////Divide???
			//Colour emitted = light->evaluate(-wi) / pmf; //FIXXX
			//Colour emitted = light->evaluate(-wi);
			float pdf;
			Colour emitted;
			light->sample(ShadingData(), sampler, emitted, pdf);

			Colour Le = emitted * fabs(Dot(wi, lightNormal)) / (pmf * pdfPosition * pdfDirection * (float)N_VPLs);

			ShadingData VPLLight(p, lightNormal);
			VPLs.emplace_back(VPLLight, Le);


			Ray ray;
			ray.init(p+(wi*EPSILON), wi);

			Colour pathThroughput(1.0f, 1.0f, 1.0f);;
			VPLTracePath(ray, pathThroughput, Le, sampler, 0);
		}
	}

	void VPLTracePath(Ray& r, Colour pathThroughput, Colour Le, Sampler* sampler, int depth) {
		if (depth > MAX_DEPTH)
		{
			return;
		}
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);

		if (shadingData.t < FLT_MAX)
		{
			//if (shadingData.bsdf->isLight() || shadingData.bsdf->isPureSpecular())
			//{
				//return;
			//}

			// Store a VPL at this intersection
			//if (shadingData.bsdf->isPureSpecular() == false) {
				VPLs.emplace_back(shadingData, pathThroughput * Le);
			//}

			float russianRouletteProbability = min(pathThroughput.Lum(), 0.9f);
			if (sampler->next() < russianRouletteProbability)
			{
				pathThroughput = pathThroughput / russianRouletteProbability;
			}
			else
			{
				return;
			}
			Vec3 wi;
			Colour indirect;
			float pdf;

			//if (shadingData.bsdf->isPureSpecular()) {
				wi = shadingData.bsdf->sample(shadingData, sampler, indirect, pdf);
				//pathThroughput = pathThroughput * indirect;

			//}
			//else {
				//wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
				//pdf = SamplingDistributions::cosineHemispherePDF(wi);
				//wi = shadingData.frame.toWorld(wi);
				//indirect = shadingData.bsdf->evaluate(shadingData, wi);
				pathThroughput = pathThroughput * indirect * fabsf(Dot(wi, shadingData.sNormal)) / pdf;
			//}


			Ray newRay;
			newRay.init(shadingData.x + (wi * EPSILON), wi);
			VPLTracePath(newRay, pathThroughput, Le, sampler, depth+1);
		}
	};

	//vpl.Le = pathThroughput * Le;
	//vpls[i].shadingData.bsdf->evaluate(vpls[i].shadingData, -wi);
	//vpls[i].Le* shadingData.bsdf->evaluate(shadingData, wi)* GTerm;

	Colour computeVPLContribution(const ShadingData& shadingData, Sampler* sampler) {
		Colour indirect(0.0f, 0.0f, 0.0f);

		for (const VPL & vpl : VPLs) {
#
			// Direction from the hit point (x) to the VPL (xi)
			Vec3 wi = vpl.shadingData.x - shadingData.x;
			wi = wi.normalize();
			float distance = wi.length();


			// Check visibility between x and xi
			if (scene->visible(shadingData.x, vpl.shadingData.x) == false) continue;

			// Compute geometry term G(xi  x)
			float cosThetaX = Dot(shadingData.sNormal, wi);
			float cosThetaXi = Dot(vpl.shadingData.sNormal, -wi);
			
			if (cosThetaX <= 0.0f || cosThetaXi <= 0.0f) continue; 

			float geometryTerm = cosThetaX * cosThetaXi / (distance * distance);

			if (geometryTerm <= 0.0f) continue;

			// BSDF at the hit point x
			Colour fr_x = shadingData.bsdf->evaluate(shadingData, wi);

			if (shadingData.bsdf->isLight() || vpl.shadingData.bsdf == nullptr) {
				indirect = indirect + (fr_x * geometryTerm  * vpl.Le);
			}
			else {
				// if on a surface
				Colour fr_xi = vpl.shadingData.bsdf->evaluate(vpl.shadingData, -wi);
				indirect = indirect + (fr_x * geometryTerm * fr_xi * vpl.Le);
			}
		}
		return indirect;
	}

	Colour pathTraceRadiosity(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler, bool canHitLight = true)
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

			Colour indirectVPL(0.0f, 0.0f, 0.0f);

			//if (!shadingData.bsdf->isPureSpecular()) {
				indirectVPL = pathThroughput * computeVPLContribution(shadingData, sampler);
				direct = direct + indirectVPL;
				//indirectVPL = computeVPLContribution(shadingData, sampler);
			//}

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
			
			//return direct;
			Colour indirect;
			float pdf;
			//Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
			Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, indirect, pdf);
			pathThroughput = pathThroughput * indirect * fabsf(Dot(wi, shadingData.sNormal)) / pdf;
			
			r.init(shadingData.x + (wi * EPSILON), wi);
			return (direct + pathTraceRadiosity(r, pathThroughput, depth + 1, sampler, shadingData.bsdf->isPureSpecular()));
		}
		return scene->background->evaluate(r.dir);
	}


	float calculateAcceptance(Colour current, Colour proposed) {
		float currentLum = current.Lum();
		float proposedLum = proposed.Lum();

		if (currentLum <= 0.0f)return 1.0f;
		if (proposedLum <= 0.0f)return 0.0f;

		return (std::min)(1.0f, proposedLum / currentLum);
	}

	int M = 100;

	Colour genPathC(PSSSampler* sampler, float&x , float&y) {

		x = sampler->nextPSSC() * film->width;
		y = sampler->nextPSSC() * film->height;

		return pathTrace(x, y, sampler);
	}

	Colour genPathP(PSSSampler* sampler, float& x, float& y) {
		x = sampler->nextPSSP() * film->width;
		y = sampler->nextPSSP() * film->height;

		return pathTrace(x, y, sampler);
	}

	float getNormalization(PSSSampler* sampler) {
		float b = 0.0f;
		for (int i = 0; i < M; i++) {
			float x = sampler->next() * film->width;
			float y = sampler->next() * film->height;
			Colour currentValue = pathTrace(x, y, sampler);
			b += currentValue.Lum();
		}
		b /= M;
		if (b <= 0.0f) b = 1.0f;
		return b;
	}

	PSSSampler *pssSampler = nullptr;

	float largeStepProb = 0.3f;

	void PSSMLTRender(int numSamples) {
		if (pssSampler == nullptr) {
			pssSampler = new PSSSampler();
		}

		float b = getNormalization(pssSampler);
		float proposedX, proposedY;
		float currentX, currentY;

		Colour currentValue = genPathC(pssSampler , currentX , currentY);

		for (int i = 0; i < numSamples; i++)
		{

			bool largeStep = pssSampler->next() < largeStepProb;
			if (largeStep) {
				// Perform a small step perturbation
				pssSampler->largeStep();
			}
			else {
				pssSampler->smallStep();
			}
			
			Colour proposed = genPathP(pssSampler , proposedX , proposedY);
			
			float a = calculateAcceptance(currentValue, proposed);

			if (largeStep) {
				film->splat(proposedX, proposedY, proposed * a / (b + largeStepProb * proposed.Lum()));
				film->splat(currentX, currentY, currentValue * (1.0f - a) / b );
			}
			else {
				film->splat(proposedX, proposedY, proposed * a / b );
				film->splat(currentX, currentY, currentValue * (1.0f - a) / b);
			}


			//Colour currentContrib = currentValue * (1.0f - a) * b / currentValue.Lum();
			//Colour proposedContrib = proposed * a * b / proposed.Lum();

			if (pssSampler->next() < a) {
				//pssSampler->currentState = pssSampler->proposedState;
				//film->splat(proposedX, proposedY, proposedContrib);
				currentValue = proposed;
				currentX = proposedX;
				currentY = proposedY;
			}
			//film->splat(currentX, currentY, currentContrib);
		}

	}

	
};