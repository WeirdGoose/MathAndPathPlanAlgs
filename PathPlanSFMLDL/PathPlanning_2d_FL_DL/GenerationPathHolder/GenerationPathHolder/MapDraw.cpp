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
	//obstacle_point quadro1;
	//obstacle_point quadro2;
	//quadro1.x = 180;
	//quadro1.y = 210;
	//quadro2.x = 290;
	//quadro2.y = 260;
	//map.create_quadro_obstacle(50, 50, quadro1, 3);
	//map.create_quadro_obstacle(50, 50, quadro2, 3);

	map.aim.x = AIM_POS_X;
	map.aim.y = AIM_POS_Y;
	map.rob_position.x = START_ROBOT_POS_X;
	map.rob_position.y = START_ROBOT_POS_Y;

	obstacle_point quadro1;
	obstacle_point quadro2;
	obstacle_point quadro3;
	quadro1.x = 70;
	quadro1.y = 400;
	quadro2.x = 250;
	quadro2.y = 250;
	quadro3.x = 445;
	quadro3.y = 100;
	map.create_quadro_obstacle(100, 100, quadro1, 2);
	map.create_quadro_obstacle(100, 100, quadro2, 2);
	map.create_quadro_obstacle(100, 100, quadro3, 2);
	obstacle_point line_p_1(400, 0);
	obstacle_point line_p_11(200, 150);
	obstacle_point line_p_2(170, 160);
	obstacle_point line_p_22(0, 270);
	obstacle_point line_p_3(200, 150);
	obstacle_point line_p_33(250, 50);
	obstacle_point line_p_4(170, 160);
	obstacle_point line_p_44(210, 80);
	obstacle_point line_p_5(250, 50);
	obstacle_point line_p_55(40, 120);
	obstacle_point line_p_6(210, 80);
	obstacle_point line_p_66(5, 150);
	obstacle_point line_p_7(40, 120);
	obstacle_point line_p_77(80, 50);
	obstacle_point line_p_8(5, 150);
	obstacle_point line_p_88(60, 40);
	//map.create_line_obstacle(line_p_1, line_p_11);
	//map.create_line_obstacle(line_p_2, line_p_22);
	map.create_line_obstacle(line_p_3, line_p_33);
	map.create_line_obstacle(line_p_4, line_p_44);
	//map.create_line_obstacle(line_p_5, line_p_55);
	//map.create_line_obstacle(line_p_6, line_p_66);
	map.create_line_obstacle(line_p_7, line_p_77);
	map.create_line_obstacle(line_p_8, line_p_88);
	obstacle_point circle_center1(70, 270);
	obstacle_point circle_center2(150, 210);
	obstacle_point circle_center3(150, 300);
	obstacle_point circle_center4(120, 50);
	obstacle_point circle_center5(260, 170);
	obstacle_point circle_center6(330, 110);
	obstacle_point circle_center7(120, 50);
	obstacle_point circle_center8(360, 170);
	obstacle_point circle_center9(430, 200);
	obstacle_point circle_center10(220, 360);
	obstacle_point circle_center11(300, 430);
	obstacle_point circle_center12(380, 360);
	obstacle_point circle_center13(340, 250);
	obstacle_point circle_center14(470, 280);
	obstacle_point circle_center15(470, 470);
	map.create_circle_obstacle(circle_center1, 30);
	map.create_circle_obstacle(circle_center2, 15);
	map.create_circle_obstacle(circle_center3, 15);
	map.create_circle_obstacle(circle_center4, 15);
	map.create_circle_obstacle(circle_center5, 16);
	map.create_circle_obstacle(circle_center6, 22);
	map.create_circle_obstacle(circle_center7, 15);
	map.create_circle_obstacle(circle_center8, 15);
	map.create_circle_obstacle(circle_center9, 30);
	map.create_circle_obstacle(circle_center10, 30);
	map.create_circle_obstacle(circle_center11, 30);
	map.create_circle_obstacle(circle_center12, 30);
	map.create_circle_obstacle(circle_center13, 20);
	map.create_circle_obstacle(circle_center14, 20);
	map.create_circle_obstacle(circle_center15, 25);

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

void draw_internal_map(sf::RenderWindow &win,
	robot_params &rob_base)
{
	sf::CircleShape Circle1(0.8);
	Circle1.setFillColor(sf::Color::Blue);
	for (unsigned int i = 0; i < rob_base.internal_map.size(); ++i)
	{
		Circle1.setPosition(rob_base.internal_map.at(i).x, rob_base.internal_map.at(i).y);
		win.draw(Circle1);
	}
}

void draw_other(sf::RenderWindow &win,
	Whole_map &map,
	robot_params &rob_base,
	std::vector<sf::CircleShape> &sens_line_space)
{
	sf::CircleShape Circle_path(3);
	sf::CircleShape Circle_rob(0.5);
	sf::CircleShape Circle_aim(15.f);
	sf::CircleShape Circle_sens_line(1.f);
	sf::CircleShape Circle_orient(7.f);
	Circle_path.setFillColor(sf::Color::Red);

	//catch position trough the wibe (proceedeng float to int)
	for (unsigned int i = 0; i < rob_base.Path.size(); ++i)
	{
		Circle_path.setPosition(rob_base.Path.at(i).x, rob_base.Path.at(i).y);
		win.draw(Circle_path);
	}

	// Draw current robot position
	Circle_rob.setFillColor(sf::Color::Red);
	Circle_rob.setPosition(rob_base.position.x, rob_base.position.y);
	win.draw(Circle_rob);
#if !DRAW_INTERNAL
	// Draw aim (it is abscent in map space)
	Circle_aim.setFillColor(sf::Color::Blue);
#else
	Circle_aim.setFillColor(sf::Color::Yellow);
#endif
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
	//obstacle_point orient = rob_base.get_orientation();
	//Circle_orient.setFillColor(sf::Color::Red);
	//Circle_orient.setPosition(orient.x, orient.y);
	//win.draw(Circle_orient);

}

void scene_movment(Whole_map &map, robot_params &rob_base, simulation &sim)
{

	sf::RenderWindow window(sf::VideoMode(map.get("width"), map.get("height")), "SFML works!");
	std::vector<sf::CircleShape> Obs;
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
		// signal to robot_logic that don't wont to be earlier than this thread (he can began now)
		synchCndVar.notify_one();
		WAIT_FOR_DRIVE(synchCndVar, uLock);
		window.clear();
#if DRAW_INTERNAL
		draw_internal_map(window, rob_base);
#else
		draw_map(window, map, Obs);
#endif
		draw_other(window, map, rob_base, lines);
		window.display();
		SIGNAL_TO_DRIVE(synchCndVar);
	}
	sim.off_simulation();
}
