#include "pch.h"
#include "CppUnitTest.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

#include <iostream>
#include "../RTBase/Geometry.h"

namespace RTTest
{
	TEST_CLASS(RTTest)
	{
	public:
		
		TEST_METHOD(TestRayIntersect)
		{
			Plane p;
			p.init(Vec3(0, 1, 1), 0);

			Ray r(Vec3(0, 10, 0), Vec3(0, -1, 0));

			float t = 0;
			Assert::IsTrue(p.rayIntersect(r, t));

			Logger::WriteMessage("Test");
		}

		TEST_METHOD(InfinteArea)
		{
			AABB test;
			float area = test.area();
			std::string output = "Area = " + std::to_string(area);
			Logger::WriteMessage(output.c_str());

			float inf = std::numeric_limits<float>::infinity();
			Assert::AreEqual(area, inf);
		}

		TEST_METHOD(TestRayInterset)
		{
			Triangle tri;
			tri.init({ Vec3(0, 0, 0), Vec3(1.0f, 0, 0), 0.f, 0.f },
				{ Vec3(1.0f, 0, 0), Vec3(1.0f, 0, 0), 0, 0 },
				{ Vec3(0, 1.0f, 0), Vec3(1.0f, 0, 0), 0 }, 0
			);

			//above the traingle
			Ray r(Vec3(0.25f, 0.25f, 1.0f), Vec3(0, 0, -1));

			float t;
			float u;
			float v;

			tri.rayIntersect(r, t, u, v);

			std::string test = "t = " + std::to_string(t) + " u = " + std::to_string(u) + " v = " + std::to_string(v);
			Logger::WriteMessage(test.c_str());

			float t2;
			float u2;
			float v2;

			tri.rayIntersect2(r, t2, u2, v2);

			test = "t2 = " + std::to_string(t2) + " u2 = " + std::to_string(u2) + " v2 = " + std::to_string(v2);
			Logger::WriteMessage(test.c_str());


			Assert::AreEqual(t, t2);
			Assert::AreEqual(u, u2);
			Assert::AreEqual(v, v2);
		}
	};
}
