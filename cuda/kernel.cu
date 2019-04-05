
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

#include <stdio.h>
#include <math.h>
#include <float.h>

#include "cuda.h"

#define CHECK_CUDA_ERROR(call)																									\
{																																\
	const cudaError_t error = call;																								\
	if (error != cudaSuccess)																									\
	{																															\
		char str[1024];																											\
		sprintf_s(str, 1024, "Cuda Error: %s:%d, cuda:%d, reason: %s\n", __FILE__, __LINE__, error, cudaGetErrorString(error));	\
		return str;																												\
	}																															\
}																																\

namespace cuda
{
#if 0
}   // indent guard
#endif

constexpr size_t BLOCK_SIZE = 32;

__device__ __host__ void updateObjectPosition(Object & object, double updateTime)
{
	object.centerX_ += object.velocityX_ * updateTime;
	object.centerY_ += object.velocityY_ * updateTime;
}

__device__ __host__ void updateObjectVelocity(Object & object, double updateTime)
{
	object.velocityX_ += object.accelerationX_ * updateTime;
	object.velocityY_ += object.accelerationY_ * updateTime;
}

__device__ __host__ bool checkCollisionObjects(const Object & obj1, const Object & obj2, double deltatime, double & collisionTime)
{
	double Dx0 = obj1.centerX_ - obj2.centerX_;
	double Dy0 = obj1.centerY_ - obj2.centerY_;

	double Dx1 = obj1.centerX_ - obj2.centerX_ + obj1.velocityX_ - obj2.velocityX_;
	double Dy1 = obj1.centerY_ - obj2.centerY_ + obj1.velocityY_ - obj2.velocityY_;

	double dDx = Dx1 - Dx0;
	double dDy = Dy1 - Dy0;

	double P = dDx * dDx + dDy * dDy;
	double Q = dDx * Dx0 + dDy * Dy0;
	double R = Dx0 * Dx0 + Dy0 * Dy0;

	double r12S = (obj1.radius_ + obj2.radius_) * (obj1.radius_ + obj2.radius_);

	if (R < r12S)
	{
		collisionTime = 0.0;
		return true;
	}

	if (P < DBL_EPSILON)
	{
		return false;
	}

	double judge = Q * Q - P * (R - r12S);

	if (judge < 0)
	{
		return false;
	}

	double aqrtJudge = sqrt(judge);
	double T1 = (-Q + judge) / P;
	double T2 = (-Q - judge) / P;
	double T = T1 < T2 ? T1 : T2;

	if (T < 0 || deltatime < T)
	{
		return false;
	}

	collisionTime = T;

	return true;
}

__device__ __host__ bool checkCollisionObjectLine(const Object & obj, const LineObject & line, double deltatime, double & collisionTime)
{
	double Nx = line.y1_ - line.y2_;
	double Ny = line.x1_ - line.x2_;

	double NN = sqrt(Nx * Nx + Ny * Ny);

	if (NN < DBL_EPSILON)
	{
		return false;
	}

	double PN = abs((obj.centerX_ - line.x1_) * Nx + (obj.centerY_ - line.y1_) * Ny);
	double VN = obj.velocityX_ * Nx + obj.velocityY_ * Ny;

	double D = PN / NN - obj.radius_;
	double dD = VN / NN;

	if (dD * dD < DBL_EPSILON)
	{
		return false;
	}

	double T = D / dD;

	if (T < 0 || deltatime < T)
	{
		return false;
	}

	collisionTime = T;

	return true;
}

__device__ __host__ void collisionProcessObjects(Object & obj1, Object & obj2, const double objectMass, const double coefficientOfRestitution)
{
	double Dx = obj2.centerX_ - obj1.centerX_;
	double Dy = obj2.centerY_ - obj1.centerY_;

	if (Dx < DBL_EPSILON && Dx < DBL_EPSILON)
	{
		return;
	}

	double nDx = Dx / sqrt(Dx * Dx + Dy * Dy);
	double nDy = Dy / sqrt(Dx * Dx + Dy * Dy);

	double VD12x = obj1.velocityX_ - obj2.velocityX_;
	double VD12y = obj1.velocityY_ - obj2.velocityY_;
	double VD21x = obj2.velocityX_ - obj1.velocityX_;
	double VD21y = obj2.velocityY_ - obj1.velocityY_;

	double S1 = VD21x * nDx + VD21y * nDy;
	double S2 = VD12x * nDx + VD12y * nDy;

	if (objectMass < DBL_EPSILON)
	{
		return;
	}

	double C1 = (objectMass / (objectMass + objectMass)) * (1 + coefficientOfRestitution) * S1;
	double C2 = (objectMass / (objectMass + objectMass)) * (1 + coefficientOfRestitution) * S2;

	obj1.velocityX_ = C1 * nDx + obj1.velocityX_;
	obj1.velocityY_ = C1 * nDy + obj1.velocityY_;
	obj2.velocityX_ = C2 * nDx + obj2.velocityX_;
	obj2.velocityY_ = C2 * nDy + obj2.velocityY_;
}

__device__ __host__ void collisionProcessObjectLine(Object & obj, const LineObject & line, const double objectMass, const double coefficientOfRestitution)
{
	double Nx = line.y1_ - line.y2_;
	double Ny = line.x1_ - line.x2_;

	if (Nx * Nx < DBL_EPSILON && Ny * Ny < DBL_EPSILON)
	{
		return;
	}

	double nNx = Nx / sqrt(Nx * Nx + Ny * Ny);
	double nNy = Ny / sqrt(Nx * Nx + Ny * Ny);

	double dotNV = obj.velocityX_ * nNx + obj.velocityY_ * nNy;

	double C = (1 + coefficientOfRestitution) * dotNV;

	obj.velocityX_ = obj.velocityX_ - C * nNx;
	obj.velocityY_ = obj.velocityY_ - C * nNy;
}

__global__ void updateObjects(double deltatime, Object* objSrc, Object* objDst, size_t objSIze, LineObject* lnObjSrc, size_t lnObjSize, const double objectMass, const double coefficientOfRestitution)
{
	const size_t objIdx = blockIdx.x * BLOCK_SIZE + threadIdx.x;
	Object obj = objSrc[objIdx];
	updateObjectVelocity(obj, deltatime);
	for (size_t i = 0; i < 4; i++)
	{
		double collisionTime = 0.0;
		if (checkCollisionObjectLine(obj, lnObjSrc[i], deltatime, collisionTime))
		{
			updateObjectPosition(obj, collisionTime);
			collisionProcessObjectLine(obj, lnObjSrc[i], objectMass, coefficientOfRestitution);
			updateObjectPosition(obj, deltatime - collisionTime);
			objDst[objIdx] = obj;
			return;
		}
	}
	const size_t halfObjSize = (objSIze + 1) / 2;
	const size_t endCheckIdx = (objIdx + halfObjSize + 1) % objSIze;
	Object otherObj;
	for (size_t i = objIdx + 1; i != endCheckIdx; i = (i + 1) % objSIze)
	{
		double collisionTime = 0.0;
		otherObj = objSrc[i];
		if (cuda::checkCollisionObjects(obj, otherObj, deltatime, collisionTime))
		{
			cuda::updateObjectPosition(obj, collisionTime);
			cuda::updateObjectPosition(otherObj, collisionTime);
			cuda::collisionProcessObjects(obj, otherObj, objectMass, coefficientOfRestitution);
			cuda::updateObjectPosition(obj, deltatime - collisionTime);
			// cuda::updateObjectPosition(otherObj, deltatime - collisionTime);
			objDst[objIdx] = obj;
			return;
		}
	}
	updateObjectPosition(obj, deltatime);
	objDst[objIdx] = obj;
	return;
}

char* gpu_update(double deltatime, Object* objects, size_t objectsSize, LineObject* lineObjects, size_t lineObjectsSize, const double objectMass, const double coefficientOfRestitution)
{
	Object* dInObjects;
	LineObject* dInLineObjects;

	Object* dOutObjects;

	CHECK_CUDA_ERROR(cudaMallocHost((void**)&dInObjects, sizeof(Object) * objectsSize));
	CHECK_CUDA_ERROR(cudaMallocHost((void**)&dInLineObjects, sizeof(LineObject) * lineObjectsSize));

	CHECK_CUDA_ERROR(cudaMallocHost((void**)&dOutObjects, sizeof(Object) * objectsSize));

	CHECK_CUDA_ERROR(cudaMemcpy(dInObjects, objects, sizeof(Object) * objectsSize, cudaMemcpyHostToDevice));
	CHECK_CUDA_ERROR(cudaMemcpy(dInLineObjects, lineObjects, sizeof(LineObject) * lineObjectsSize, cudaMemcpyHostToDevice));

	dim3 block(BLOCK_SIZE);
	dim3 grid((objectsSize + block.x - 1) / block.x);

	updateObjects <<<grid, block>>>(deltatime, dInObjects, dOutObjects, objectsSize, dInLineObjects, lineObjectsSize, objectMass, coefficientOfRestitution);

	CHECK_CUDA_ERROR(cudaDeviceSynchronize());
	
	CHECK_CUDA_ERROR(cudaMemcpy(objects, dOutObjects, sizeof(Object) * objectsSize, cudaMemcpyDeviceToHost));

	CHECK_CUDA_ERROR(cudaFreeHost((void*)dInObjects));
	CHECK_CUDA_ERROR(cudaFreeHost((void*)dInLineObjects));
	CHECK_CUDA_ERROR(cudaFreeHost((void*)dOutObjects));

	return nullptr;
}

}
