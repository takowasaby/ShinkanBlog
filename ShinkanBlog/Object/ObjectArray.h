#pragma once

#include <vector>
#include <array>
#include <cuda.h>

class ObjectArray
{
public:
	ObjectArray(size_t size);

	void update(double deltatime);
	void draw() const;

private:
	using Object = cuda::Object;
	using LineObject = cuda::LineObject;

	void cpu_update(double deltatime);

	std::vector<Object> objects_;
	std::array<LineObject, 4> windowEdges_;
};