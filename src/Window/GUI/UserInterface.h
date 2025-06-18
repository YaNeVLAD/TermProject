#ifndef USER_INTERFACE_H
#define USER_INTERFACE_H

#include <functional>

#include "../../Application/AppState.h"

namespace sf
{
class Event;
class Clock;
} // namespace sf

namespace tp
{
class Window;

class UserInterface
{
public:
	void onInit(Window* window) const;
	void onUpdate(Window* window, sf::Clock& clock) const;
	void onDisplay(Window* window) const;
	void onRender(Window* window, AppState& state, std::function<void()> onObstacleEditCallback) const;
	void onDestroy(Window* window) const;
	void HandleEvent(const sf::Event& event) const;
};
} // namespace tp

#endif
