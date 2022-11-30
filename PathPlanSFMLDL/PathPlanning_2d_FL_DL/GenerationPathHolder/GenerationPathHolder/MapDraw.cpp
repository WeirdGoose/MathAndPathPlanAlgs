#include <list>    
#include <SFML/Graphics.hpp>
#include "MapDraw.h"
#include <thread>
#include <mutex>
#include <windows.h>
#include "gen_pars.h"
#include <synchapi.h>
#include <condition_variable>

extern HANDLE sleepTimerMutex;
extern std::mutex synchMutex;
extern std::mutex synchMutex2;
extern std::condition_variable synchCndVar;


void set_map(Whole_map &map)
{
	obstacle_point quadro1;
	obstacle_point quadro2;
	quadro1.x = 180;
	quadro1.y = 210;
	quadro2.x = 290;
	quadro2.y = 260;
	map.create_quadro_obstacle(50, 50, quadro1, 3);
	map.create_quadro_obstacle(50, 50, quadro2, 3);

	map.aim.x = AIM_POS_X;
	map.aim.y = AIM_POS_Y;
	map.rob_position.x = START_ROBOT_POS_X;
	map.rob_position.y = START_ROBOT_POS_Y;

}

void draw_map(sf::RenderWindow &win,
	Whole_map &map,
	std::vector<sf::CircleShape> &obs_space)
{
	sf::CircleShape Circle1(0.5);
	Circle1.setFillColor(sf::Color::Green);

	for (unsigned int i = 0; i < map.short_obs.size(); ++i)
	{
		Circle1.setPosition(map.short_obs.at(i).x, map.short_obs.at(i).y);
		win.draw(Circle1);
	}
}

void draw_other(sf::RenderWindow &win,
	Whole_map &map,
	robot_params &rob_base,
	std::vector<sf::CircleShape> &sens_line_space,
	std::vector<map_point> &path_space)
{
	sf::CircleShape Circle_path(0.5);
	sf::CircleShape Circle_rob(0.5);
	sf::CircleShape Circle_aim(3.f);
	sf::CircleShape Circle_sens_line(1.f);
	sf::CircleShape Circle_orient(1.f);
	Circle_path.setFillColor(sf::Color::Red);
	unsigned int k = 0;

	//catch position trough the wibe (proceedeng float to int)
	for (unsigned int i = 0; i < path_space.size(); ++i)
	{
		Circle_path.setPosition(path_space.at(i).x, path_space.at(i).y);
		win.draw(Circle_path);
		k++;
	}


	// Draw current robot position
	Circle_rob.setFillColor(sf::Color::Red);
	Circle_rob.setPosition(rob_base.position.x, rob_base.position.y);
	win.draw(Circle_rob);

	// Draw aim (it is abscent in map space)
	Circle_aim.setFillColor(sf::Color::Blue);
	Circle_aim.setPosition(rob_base.aim.x, rob_base.aim.y);
	win.draw(Circle_aim);

	// Draw sensor lines (there are abscent on map)
	Circle_sens_line.setFillColor(sf::Color::White);

	for (unsigned int i = 0; i < rob_base.rob_lines_num; ++i)
	{
		Circle_sens_line.setPosition(rob_base.at(i).x, rob_base.at(i).y);
		sens_line_space[i] = Circle_sens_line;
		win.draw(Circle_sens_line);
	}
	obstacle_point orient = rob_base.get_orientation();
	Circle_orient.setFillColor(sf::Color::Red);
	Circle_orient.setPosition(orient.x, orient.y);
	win.draw(Circle_orient);

}

void scene_movment(Whole_map &map, robot_params &rob_base, simulation &sim)
{

	sf::RenderWindow window(sf::VideoMode(map.get("width"), map.get("height")), "SFML works!");
	std::vector<sf::CircleShape> Obs;
	std::vector<map_point> Path;
	std::vector<sf::CircleShape> lines;
	unsigned long size = map.obstacles_weight;
	map_point robot_pos;
	std::unique_lock<std::mutex> uLock(synchMutex2);

	Obs.resize(size);
	lines.resize(LINES_NUMBER);

	while (window.isOpen())
	{

		sf::Event event;
		while (window.pollEvent(event))
		{
			if (event.type == sf::Event::Closed)
				window.close();
		}
		//synchCndVar.wait(uLock);
		WAIT_FOR_DRIVE(synchCndVar, uLock);
		window.clear();
		draw_map(window, map, Obs);
		
		robot_pos.x = rob_base.position.x;
		robot_pos.y = rob_base.position.y;
		if (map.at(robot_pos.x, robot_pos.y) != ROBOT_PATH_MAP_CHAR)
		{
			// save to map last position
			Path.push_back(robot_pos);
			map.at(robot_pos.x, robot_pos.y) = ROBOT_PATH_MAP_CHAR;
		}
		
		draw_other(window, map, rob_base, lines, Path);
		window.display();
		SIGNAL_TO_DRIVE(synchCndVar);
	}
	sim.off_simulation();
}
