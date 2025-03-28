#pragma once

#include <functional>
#include <memory>
#include <string>

#include "Shapes.h"

#include "SFML/Graphics.hpp"

// namespace sf
//{
// class RenderWindow;
// }

namespace tp
{
class Window
{
public:
	Window(unsigned width, unsigned height, const std::string& name);

	void Draw(const tp::Shapes::Segment& segment, sf::Color = sf::Color::White);
	void Draw(const tp::Shapes::Polygon& polygon);
	void Render();
	void PollEvents();

	void SetMouseCallback(std::function<void(int, int, int)> callback);

	bool IsRunning() const;

	void Close();

private:
	std::function<void(int, int, int)> m_Callback = nullptr;

	bool m_Running = true;

	unsigned m_Width;
	unsigned m_Height;

	std::shared_ptr<sf::RenderWindow> m_Window = nullptr;
};

} // namespace tp
