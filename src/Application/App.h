#ifndef APP_H
#define APP_H

#include "../Algorithm/Voronoi/Voronoi.h"
#include "../Window/GUI/UserInterface.h"
#include "../Window/Window.h"
#include "AppState.h"

namespace tp
{
class App
{
public:
	App();

	void Run();

private:
	AppState m_state;

	UserInterface m_gui;
	Window m_window;
	Voronoi m_voronoi;
	PathFinder m_pathFinder;

	void OnMouseClick(int button, int x, int y);
	void OnKeyPressed(keyboard::Key key);
};
} // namespace tp

#endif
