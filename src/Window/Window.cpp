//
// Created by User on 15.04.2025.
//

#include "Window.h"

#include "SFML/Graphics.hpp"

using namespace tp;
using namespace shapes;

Window::Window(unsigned width, unsigned height, const std::string& title)
	: m_window(std::make_shared<sf::RenderWindow>(sf::VideoMode({ width, height }), title))
{
}

Window::Window(Vec2D size, const std::string& title)
	: Window(static_cast<unsigned>(size.x), static_cast<unsigned>(size.y), title)
{
}

void Window::Draw(const Point& point, Color color) const
{
	static float POINT_RADIUS = 5;
	auto _color = sf::Color(color.ToInt());
	sf::CircleShape circle(POINT_RADIUS);
	circle.setOrigin({ POINT_RADIUS / 2, POINT_RADIUS / 2 });
	circle.setPosition({ point.x, point.y });
	circle.setFillColor(sf::Color(_color));
	m_window->draw(circle);
}

void Window::Draw(const Segment& segment, Color color) const
{
	auto _color = sf::Color(color.ToInt());
	sf::Vertex line[] = {
		{ sf::Vector2f(segment.p1.x, segment.p1.y), _color },
		{ sf::Vector2f(segment.p2.x, segment.p2.y), _color },
	};
	m_window->draw(line, std::size(line), sf::PrimitiveType::LineStrip);
}

void Window::Draw(const Segment& segment, Color color, float thickness) const
{
	if (thickness <= 1.0f)
	{
		Draw(segment, color);
		return;
	}

	auto _color = sf::Color(color.ToInt());

	sf::Vector2f p1(segment.p1.x, segment.p1.y);
	sf::Vector2f p2(segment.p2.x, segment.p2.y);

	sf::Vector2f direction = p2 - p1;
	float length = std::sqrt(direction.x * direction.x + direction.y * direction.y);

	if (length == 0)
	{
		Draw(Point(p1.x, p1.y), color);
		return;
	}

	sf::Vector2f unitDirection = direction / length;
	sf::Vector2f unitPerpendicular(-unitDirection.y, unitDirection.x);

	sf::Vector2f offset = unitPerpendicular * (thickness / 2.0f);

	sf::ConvexShape thickLine;
	thickLine.setPointCount(4);
	thickLine.setPoint(0, p1 - offset);
	thickLine.setPoint(1, p2 - offset);
	thickLine.setPoint(2, p2 + offset);
	thickLine.setPoint(3, p1 + offset);
	thickLine.setFillColor(_color);

	m_window->draw(thickLine);
}

void Window::Draw(const Polygon& polygon, Color color, bool isFilled) const
{
	size_t size = polygon.size();

	if (!isFilled)
	{
		std::vector<Segment> segments;
		segments.reserve(size * 2);
		for (size_t i = 0; i < size; i++)
		{
			segments.emplace_back(polygon[i], polygon[(i + 1) % size]);
		}

		for (const auto& segment : segments)
		{
			Draw(segment, color);
		}
	}
	else
	{
		auto _color = sf::Color(color.ToInt());
		sf::ConvexShape shape;
		shape.setFillColor(_color);
		shape.setPointCount(size);
		for (size_t i = 0; i < size; ++i)
		{
			shape.setPoint(i, { polygon[i].x, polygon[i].y });
		}
		m_window->draw(shape);
	}
}

void Window::Render(Color backgroundColor) const
{
	m_window->display();
	m_window->clear(sf::Color(backgroundColor.ToInt()));
}

void Window::PollEvents() const
{
	sf::Event event;
	m_window->pollEvent(event);
	if (event.type == sf::Event::Closed)
	{
		m_window->close();
	}
	if (event.type == sf::Event::KeyPressed)
	{
		auto key = static_cast<keyboard::Key>(event.key.code);
		if (m_onKeyPressedCallback)
		{
			m_onKeyPressedCallback(key);
		}
	}
	if (event.type == sf::Event::MouseButtonPressed)
	{
		auto pos = m_window->mapPixelToCoords({ event.mouseButton.x, event.mouseButton.y });
		auto button = event.mouseButton.button;
		m_onClickCallback(static_cast<int>(button), pos.x, pos.y);
	}
	if (event.type == sf::Event::Resized)
	{
		sf::Vector2f newSize(event.size.width, event.size.height);
		sf::View view = m_window->getView();
		view.setSize(newSize);
		m_window->setView(view);
	}
}

bool Window::IsRunning() const
{
	return m_window->isOpen();
}

void Window::Close() const
{
	m_window->close();
}

void tp::Window::Clear(Color color) const
{
	m_window->clear(sf::Color{ color.ToInt() });
}

void tp::Window::Display() const
{
	m_window->display();
}

void tp::Window::HandleEvent(const sf::Event& event) const
{
	if (event.type == sf::Event::Closed)
	{
		m_window->close();
	}
	if (event.type == sf::Event::KeyPressed)
	{
		auto key = static_cast<keyboard::Key>(event.key.code);
		if (m_onKeyPressedCallback)
		{
			m_onKeyPressedCallback(key);
		}
	}
	if (event.type == sf::Event::MouseButtonPressed)
	{
		auto pos = m_window->mapPixelToCoords({ event.mouseButton.x, event.mouseButton.y });
		auto button = event.mouseButton.button;
		m_onClickCallback(static_cast<int>(button), pos.x, pos.y);
	}
	if (event.type == sf::Event::Resized)
	{
		sf::Vector2f newSize(event.size.width, event.size.height);
		sf::View view = m_window->getView();
		view.setSize(newSize);
		m_window->setView(view);
	}
}

void Window::SetMouseCallback(MouseCallbackFunc&& callback)
{
	m_onClickCallback = std::forward<MouseCallbackFunc>(callback);
}

void Window::SetKeyboardCallback(KeyboardCallbackFunc&& callback)
{
	m_onKeyPressedCallback = std::forward<KeyboardCallbackFunc>(callback);
}

BorderRect Window::GetBounds() const
{
	auto viewCenter(m_window->getView().getCenter());
	auto viewSize(m_window->getView().getSize());
	float left = viewCenter.x - viewSize.x / 2;
	float top = viewCenter.y - viewSize.y / 2;
	float right = viewCenter.x + viewSize.x / 2;
	float bottom = viewCenter.y + viewSize.y / 2;

	return { left, top, right, bottom };
}
