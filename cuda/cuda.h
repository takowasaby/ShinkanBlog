#pragma once

namespace cuda
{
#if 0
}   // indent guard
#endif


#ifdef DLL_EXPORT
__declspec(dllexport) struct Object
{
	double radius_;
	double centerX_;
	double centerY_;
	double velocityX_;
	double velocityY_;
	double accelerationX_;
	double accelerationY_;
};
__declspec(dllexport) struct LineObject
{
	double x1_;
	double y1_;

	double x2_;
	double y2_;
};
__declspec(dllexport) void updateObjectPosition(Object & object, double updateTime);
__declspec(dllexport) void updateObjectVelocity(Object & object, double updateTime);
__declspec(dllexport) bool checkCollisionObjects(const Object & obj1, const Object & obj2, double deltatime, double & collisionTime);
__declspec(dllexport) bool checkCollisionObjectLine(const Object & obj, const LineObject & line, double deltatime, double & collisionTime);
__declspec(dllexport) void collisionProcessObjects(Object & obj1, Object & obj2, const double objectMass, const double coefficientOfRestitution);
__declspec(dllexport) void collisionProcessObjectLine(Object & obj, const LineObject & line, const double objectMass, const double coefficientOfRestitution);
__declspec(dllexport) char* gpu_update(double deltatime, Object* objects, size_t objectsSize, LineObject* lineObjects, size_t lineObjectsSize, const double objectMass, const double coefficientOfRestitution);
#else
__declspec(dllimport) struct Object
{
	double radius_;
	double centerX_;
	double centerY_;
	double velocityX_;
	double velocityY_;
	double accelerationX_;
	double accelerationY_;
};
__declspec(dllimport) struct LineObject
{
	double x1_;
	double y1_;

	double x2_;
	double y2_;
};
__declspec(dllimport) void updateObjectPosition(Object & object, double updateTime);
__declspec(dllimport) void updateObjectVelocity(Object & object, double updateTime);
__declspec(dllimport) bool checkCollisionObjects(const Object & obj1, const Object & obj2, double deltatime, double & collisionTime);
__declspec(dllimport) bool checkCollisionObjectLine(const Object & obj, const LineObject & line, double deltatime, double & collisionTime);
__declspec(dllimport) void collisionProcessObjects(Object & obj1, Object & obj2, const double objectMass, const double coefficientOfRestitution);
__declspec(dllimport) void collisionProcessObjectLine(Object & obj, const LineObject & line, const double objectMass, const double coefficientOfRestitution);
__declspec(dllimport) char* gpu_update(double deltatime, Object* objects, size_t objectsSize, LineObject* lineObjects, size_t lineObjectsSize, const double objectMass, const double coefficientOfRestitution);
#endif

}