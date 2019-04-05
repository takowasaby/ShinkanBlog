#include "ObjectArray.h"
#include "../Define.h"

#include <Siv3D.hpp>

#include <random>
#include <Windows.h>
#include <locale>
#include <ctime>

#ifdef MEASURE_TIME
#include <chrono>
#include <cstdio>
#endif

ObjectArray::ObjectArray(size_t size)
{
	std::random_device rnd;
	std::mt19937_64 mt((int)time(0));
	std::uniform_int_distribution randX(static_cast<int>(OBJECT_RADIUS), static_cast<int>(WINDOW_WIDTH - OBJECT_RADIUS));
	std::uniform_int_distribution randY(static_cast<int>(OBJECT_RADIUS), static_cast<int>(WINDOW_HEIGHT - OBJECT_RADIUS));

	objects_.resize(size);

	for (int i = 0; i < static_cast<int>(size); i++)
	{
		objects_[i].radius_ = OBJECT_RADIUS;
		objects_[i].centerX_ = randX(mt);
		objects_[i].centerY_ = randY(mt);
		objects_[i].velocityX_ = 0.0;
		objects_[i].velocityY_ = 0.0;
		objects_[i].accelerationX_ = 0.0;
		objects_[i].accelerationY_ = GRAVITATIONAL_ACCELERATION;
	}

	for (int i = 0; i < 4; i++)
	{
		windowEdges_[i].x1_ = WINDOW_WIDTH * (i / 2);
		windowEdges_[i].y1_ = WINDOW_HEIGHT * (i / 2);
		windowEdges_[i].x2_ = WINDOW_WIDTH * (i % 2);
		windowEdges_[i].y2_ = WINDOW_HEIGHT * ((i + 1) % 2);
	}
}

void ObjectArray::update(double deltatime)
{
#ifdef MEASURE_TIME
	std::chrono::high_resolution_clock::time_point begin = std::chrono::high_resolution_clock::now();
#endif

#ifdef USE_GPU
	char* error = cuda::gpu_update(deltatime, objects_.data(), objects_.size(), windowEdges_.data(), windowEdges_.size(), OBJECT_MASS, COEFFICIENT_OF_RESTITUTION);
	if (error != nullptr) 
	{
		wchar_t debugString[1024];
		size_t len = 0;
		errno_t err = 0;
		err = mbstowcs_s(&len, debugString, 1024, error, _TRUNCATE);
		OutputDebugString(debugString);
		System::Exit();
	}
#else
	cpu_update(deltatime);
#endif

#ifdef MEASURE_TIME
	FILE* file;
	fopen_s(&file, PUT_TIMES_FILENAME.c_str(), "a");
	if (file == nullptr)
	{
		return;
	}
	std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> elapsed_time = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end - begin);
	fprintf_s(file, "%lf\n", elapsed_time.count());
	fclose(file);
#endif
}

void ObjectArray::draw() const
{
	for (const Object& object : objects_)
	{
		s3d::Circle(object.centerX_, object.centerY_, object.radius_).draw(s3d::ColorF(0.7, 0.3, 0.0));
	}
}

void ObjectArray::cpu_update(double deltatime)
{
	size_t objectSize = objects_.size();
	std::vector<bool> objectCollisionFlag(objectSize);

	for (size_t i = 0; i < objectSize; i++)
	{
		cuda::updateObjectVelocity(objects_[i], deltatime);
	}

	for (size_t i = 0; i < objectSize; i++)
	{
		if (objectCollisionFlag[i]) continue;
		for (size_t j = 0; j < 4; j++)
		{
			double collisionTime = 0.0;
			if (cuda::checkCollisionObjectLine(objects_[i], windowEdges_[j], deltatime, collisionTime))
			{
				objectCollisionFlag[i] = true;
				cuda::updateObjectPosition(objects_[i], collisionTime);
				cuda::collisionProcessObjectLine(objects_[i], windowEdges_[j], OBJECT_MASS, COEFFICIENT_OF_RESTITUTION);
				cuda::updateObjectPosition(objects_[i], deltatime - collisionTime);
			}
		}
	}

	for (size_t i = 0; i < objectSize - 1; i++)
	{
		if (objectCollisionFlag[i]) continue;
		for (size_t j = i + 1; j < objectSize; j++)
		{
			if (objectCollisionFlag[j]) continue;
			double collisionTime = 0.0;
			if (cuda::checkCollisionObjects(objects_[i], objects_[j], deltatime, collisionTime))
			{
				objectCollisionFlag[i] = true;
				objectCollisionFlag[j] = true;
				cuda::updateObjectPosition(objects_[i], collisionTime);
				cuda::updateObjectPosition(objects_[j], collisionTime);
				cuda::collisionProcessObjects(objects_[i], objects_[j], OBJECT_MASS, COEFFICIENT_OF_RESTITUTION);
				cuda::updateObjectPosition(objects_[i], deltatime - collisionTime);
				cuda::updateObjectPosition(objects_[j], deltatime - collisionTime);
			}
		}
	}

	for (size_t i = 0; i < objectSize; i++)
	{
		if (objectCollisionFlag[i]) continue;
		cuda::updateObjectPosition(objects_[i], deltatime);
	}
}