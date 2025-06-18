#ifndef IGUI_H
#define IGUI_H

namespace tp
{
class Window;

class IGui
{
public:
	virtual void onInit(Window* window) = 0;
	virtual void onUpdate(Window* window) = 0;
	virtual void onRender(Window* window) = 0;
	virtual void onDestroy(Window* window) = 0;
};
} // namespace tp

#endif
