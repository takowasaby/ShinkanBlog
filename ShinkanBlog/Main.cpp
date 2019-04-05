#include <Siv3D.hpp>
#include "Define.h"
#include "Object/ObjectArray.h"
#include "cuda.h"

void Main()
{
	Window::Resize(WINDOW_WIDTH, WINDOW_HEIGHT);

	Graphics::SetBackground(ColorF(0.8, 0.9, 1.0));

	ObjectArray objectArray(NUMBER_OF_OBJECT);

	while (System::Update())
	{
		objectArray.update(System::DeltaTime());
		objectArray.draw();
	}
}
