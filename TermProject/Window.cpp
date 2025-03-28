#include "Window.h"

#include "SFML/Graphics.hpp"

tp::Window::Window(unsigned width, unsigned height, const std::string& name)
	: m_Width(width)
	, m_Height(height)
	, m_Window(std::make_shared<sf::RenderWindow>(sf::VideoMode(width, height), name))
{
}

void tp::Window::Draw(const tp::Shapes::Segment& segment, sf::Color color)
{
	sf::Vertex line[] = {
		{ sf::Vector2f(segment.p1.x, segment.p1.y), color },
		{ sf::Vector2f(segment.p2.x, segment.p2.y), color },
	};
	m_Window->draw(line, std::size(line), sf::Lines);
}

void tp::Window::Draw(const tp::Shapes::Polygon& polygon)
{
	size_t size = polygon.size();

	sf::ConvexShape shape;
	shape.setFillColor(sf::Color::Green);
	shape.setPointCount(size);
	for (size_t i = 0; i < size; ++i)
	{
		shape.setPoint(i, { polygon[i].x, polygon[i].y });
	}

	m_Window->draw(shape);
}

void tp::Window::Render()
{
	m_Window->display();
	m_Window->clear();
}

void tp::Window::PollEvents()
{
	sf::Event event;
	while (m_Window->pollEvent(event))
	{
		if (event.type == sf::Event::Closed)
		{
			m_Window->close();
			m_Running = false;
		}

		if (event.type == sf::Event::MouseButtonPressed)
		{
			m_Callback(static_cast<int>(event.mouseButton.button), event.mouseButton.x, event.mouseButton.y);
		}
	}
}

void tp::Window::SetMouseCallback(std::function<void(int, int, int)> callback)
{
	m_Callback = std::move(callback);
}

bool tp::Window::IsRunning() const
{
	return m_Running;
}

void tp::Window::Close()
{
	m_Window->close();
	m_Running = false;
}
