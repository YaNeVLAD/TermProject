//
// Created by User on 15.04.2025.
//

#ifndef WINDOW_H
#define WINDOW_H

#include "Keyboard.hpp"

#include <functional>
#include <memory>
#include <string>

#include "../Utility/Color/Color.h"
#include "../Utility/Shapes.hpp"

namespace sf
{
class RenderWindow;
}

namespace tp
{
class Window
{
public:
	using KeyboardCallbackFunc = std::function<void(keyboard::Key)>;
	using MouseCallbackFunc = std::function<void(int button, int x, int y)>;

	Window(unsigned width, unsigned height, const std::string& title);
	Window(shapes::Vec2D size, const std::string& title);

	void Draw(const shapes::Point& point, Color color = Color::White) const;
	void Draw(const shapes::Segment& segment, Color color = Color::White) const;
	void Draw(const shapes::Polygon& polygon, Color color = Color::Green, bool isFilled = true) const;

	void Render(Color backgroundColor = Color::Black) const;
	void PollEvents() const;

	[[nodiscard]] bool IsRunning() const;

	void Close() const;

	void SetMouseCallback(MouseCallbackFunc&& callback);

	void SetKeyboardCallback(KeyboardCallbackFunc&& callback);

	shapes::BorderRect GetBounds() const;

private:
	KeyboardCallbackFunc m_onKeyPressedCallback = nullptr;
	MouseCallbackFunc m_onClickCallback = nullptr;

	std::shared_ptr<sf::RenderWindow> m_window;
};
} // namespace tp

#endif // WINDOW_H
