#include <list>    
#include <SFML/Graphics.hpp>
#include "MapDraw.h"
#include <thread>
#include <mutex>
#include <windows.h>
#include "gen_pars.h"
#include <synchapi.h>
#include <condition_variable>
#include "genetic_algh.h"
#include <cstdlib>

extern HANDLE sleepTimerMutex;
extern std::mutex synchMutex;
extern std::mutex synchMutex2;
extern std::mutex synchMutex3;
extern std::condition_variable synchCndVar;
extern std::condition_variable synchMainDraw;


void debug_term_cmd(Whole_map& map, std::vector<robot_params>& rob_gen, simulation& sim1)
{
	std::string cmd_str;
	int index = 0;
	std::string nl_str = "nl";
	std::string draw_str = "draw";
	std::string help_str = "help";

	if (!sim1.simulation_state())
		while (!sim1.simulation_state()) Sleep(1); 
	while (sim1.simulation_state())
	{
		std::cin >> cmd_str;
		if (!strcmp(cmd_str.c_str(), draw_str.c_str()))
			synchCndVar.notify_one();
		else if (cmd_str.find(nl_str, index) == 0)
			draw_obs_vars(rob_gen.at(std::atoi(cmd_str.substr(nl_str.size(), cmd_str.size() - nl_str.size()).c_str())).genes.iodsv);
		else if (!strcmp(cmd_str.c_str(), help_str.c_str()))
			std::cout << "type \"" << draw_str << "\" to draw current bots position \n"
			<< "type \"" << nl_str << "[rob identificator]\" to to get fuzzy variables \n";
	}
}


void start_position_rules(std::vector<obstacle_point> &robs_positions, std::vector<_angle_type>& orientation_angle)
{
	rob_pop_type_ idx = 0;
	robs_positions.resize(GEN_POPULATION);
	orientation_angle.resize(GEN_POPULATION);
	uint8_t diff_x = 2;
	uint8_t diff_y = 0;
#if ROBS_DIFF_POS
	for (auto &position : robs_positions)
	{
		if (position.x + (idx)*diff_x < MAX_MAP_SIZE_X)
			position.x = START_ROBOT_POS_X + (idx)*diff_x;
		else if (position.x - (idx)*diff_x > 0)
			position.x = START_ROBOT_POS_X - (idx)*diff_x;
		else
			throw std::overflow_error("too big number of robots for x bias\n");

		if (position.y + (idx)*diff_y < MAX_MAP_SIZE_Y)
			position.y = START_ROBOT_POS_Y + (idx)*diff_y;
		else if (position.y - (idx)*diff_y > 0)
			position.y = START_ROBOT_POS_Y - (idx)*diff_y;
		else
			throw std::overflow_error("too big number of robots for y bias\n");
		orientation_angle.at(idx) = 0;
		idx++;
		cout << "rob " << (uint32_t)idx << " position x  " << position.x << " position y " << position.y << "\n";
	}
#else
	for (auto &position : robs_positions)
	{
		position.x = START_ROBOT_POS_X;
		position.y = START_ROBOT_POS_Y;
		orientation_angle.at(idx) = 0;
		idx++;
	}
#endif
}

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

	start_position_rules(map.robs_positions, map.orientation_angle);
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
	std::vector<sf::CircleShape> &sens_line_space)
{
	sf::CircleShape Circle_path(0.5);
	sf::CircleShape Circle_rob(0.5);
	sf::CircleShape Circle_aim(3.f);
	sf::CircleShape Circle_sens_line(1.f);
	sf::CircleShape Circle_orient(1.f);
	Circle_path.setFillColor(sf::Color::Yellow);

	//catch position trough the wibe (proceedeng float to int)
	for (unsigned int i = 0; i < rob_base.path.size(); ++i)
	{
		Circle_path.setPosition(rob_base.path.at(i).x, rob_base.path.at(i).y);
		win.draw(Circle_path);
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
	Circle_orient.setFillColor(sf::Color::Magenta);
	Circle_orient.setPosition(orient.x, orient.y);
	win.draw(Circle_orient);

}

void scene_movment(Whole_map &map, std::vector<robot_params>& rob_gen, simulation &sim)
{

	sf::RenderWindow window(sf::VideoMode(map.get("width"), map.get("height")), "SFML works!");
	std::vector<sf::CircleShape> Obs;
	std::vector<map_point> Path;
	std::vector<sf::CircleShape> lines;
	unsigned long size = map.obstacles_weight;
	map_point robot_pos;
	std::unique_lock<std::mutex> uLock(synchMutex2);

	Obs.resize(size);
	//Path.resize(rob_gen.at(0).path_mem_size);
	lines.resize(LINES_NUMBER);

	while (window.isOpen())
	{

		sf::Event event;
		while (window.pollEvent(event))
		{
			if (event.type == sf::Event::Closed)
				window.close();
		}		
		synchMainDraw.notify_one();
		WAIT_FOR_DRIVE(synchCndVar, uLock);
		window.clear();
		draw_map(window, map, Obs);
		
		for (auto& rob_base : rob_gen)
			draw_other(window, map, rob_base, lines);
		window.display();
		SIGNAL_TO_DRIVE(synchCndVar);
	}
	sim.off_simulation();
}

