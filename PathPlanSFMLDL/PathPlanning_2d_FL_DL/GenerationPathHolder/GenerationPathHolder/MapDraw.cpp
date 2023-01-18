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
extern std::mutex drawRobMut;
extern std::mutex mainRobMut;
extern std::mutex RobDrawMut;
extern std::mutex mainDrawMut;
extern std::condition_variable synchCndVar;
extern std::condition_variable synchMainDraw;
extern std::condition_variable generationEnd;



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
		orientation_angle.at(idx) = M_PI + atan2((position.y - AIM_POS_Y), (position.x - AIM_POS_X));
		idx++;
	}
#endif
}

void set_map(Whole_map &map)
{
	//obstacle_point quadro1;
	//obstacle_point quadro2;
	//quadro1.x = 580;
	//quadro1.y = 610;
	//quadro2.x = 290;
	//quadro2.y = 260;
	//
	//map.create_quadro_obstacle(100, 100, quadro1, 3);
	//std::cout << "map p 1 " << (int)map.at(910, 920) << std::endl;
	//
	//map.create_quadro_obstacle(100, 100, quadro2, 3);

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

	obstacle_point line_p_1	(400, 0);
	obstacle_point line_p_11(200, 150);
	obstacle_point line_p_2	(170, 160);
	obstacle_point line_p_22(0, 270);
	obstacle_point line_p_3	(200, 150);
	obstacle_point line_p_33(250, 50);
	obstacle_point line_p_4	(170, 160);
	obstacle_point line_p_44(210, 80);
	obstacle_point line_p_5	(250, 50);
	obstacle_point line_p_55(40, 120);
	obstacle_point line_p_6	(210, 80);
	obstacle_point line_p_66(5, 150);
	obstacle_point line_p_7	(40, 120);
	obstacle_point line_p_77(80, 50);
	obstacle_point line_p_8	(5, 150);
	obstacle_point line_p_88(60, 40);

	map.create_line_obstacle(line_p_1, line_p_11);
	map.create_line_obstacle(line_p_2, line_p_22);
	map.create_line_obstacle(line_p_3, line_p_33);
	map.create_line_obstacle(line_p_4, line_p_44);
	map.create_line_obstacle(line_p_5, line_p_55);
	map.create_line_obstacle(line_p_6, line_p_66);
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


	map.aim.x = AIM_POS_X;
	map.aim.y = AIM_POS_Y;

	start_position_rules(map.robs_positions, map.orientation_angle);

}

void draw_map(sf::RenderWindow &win,
	Whole_map &map,
	std::vector<sf::CircleShape> &obs_space)
{

#if DRAW_WITH_POINTS
	sf::CircleShape Circle1(0.5);
	Circle1.setFillColor(sf::Color::Green);

	for (unsigned int i = 0; i < map.short_obs.size(); ++i)
	{
		Circle1.setPosition(map.short_obs.at(i).x, map.short_obs.at(i).y);
		win.draw(Circle1);
	}
#else
	for (auto& line: map.lines_on_map)
	{
		sf::VertexArray line_sf(sf::LinesStrip, 2);
		line_sf[0].position = sf::Vector2f(line.point1.x, line.point1.y);
		line_sf[1].position = sf::Vector2f(line.point2.x, line.point2.y);
		line_sf[0].color = sf::Color::Green;
		line_sf[1].color = sf::Color::Green;
		win.draw(line_sf);
	}
	for (auto& circle : map.circles_on_map)
	{
		sf::CircleShape circle_sf;
		circle_sf.setFillColor(sf::Color::Green);
		circle_sf.setRadius(circle.radius);
		circle_sf.setPosition(circle.center.x, circle.center.y);
		win.draw(circle_sf);
	}
#endif
}

void draw_other(sf::RenderWindow &win,
	Whole_map &map,
	robot_params &rob_base,
	std::vector<sf::CircleShape> &sens_line_space)
{
	sf::CircleShape Circle_path(0.5);
	sf::CircleShape Circle_rob(0.5);
	sf::CircleShape Circle_aim(3.f);
	sf::CircleShape Circle_sens_line(0.5f);
	sf::CircleShape Circle_orient(0.5f);
	Circle_path.setFillColor(sf::Color::Yellow);

	//catch position trough the wibe (proceedeng float to int)
	for (unsigned int i = 0; i < rob_base.path.size(); ++i)
	{
		Circle_path.setPosition(rob_base.path.at(i).x, rob_base.path.at(i).y);
		win.draw(Circle_path);
	}


	// Draw current robot position
	Circle_rob.setFillColor(sf::Color::Red);
	Circle_rob.setPosition(rob_base.position.x , rob_base.position.y);
	win.draw(Circle_rob);

	// Draw aim (it is abscent in map space)
	Circle_aim.setFillColor(sf::Color::Blue);
	Circle_aim.setPosition(rob_base.aim.x , rob_base.aim.y );
	win.draw(Circle_aim);

	// Draw sensor lines (there are abscent on map)
	Circle_sens_line.setFillColor(sf::Color::White);

	for (unsigned int i = 0; i < rob_base.rob_lines_num; ++i)
	{
		Circle_sens_line.setPosition(rob_base.at(i).x , rob_base.at(i).y );
		sens_line_space[i] = Circle_sens_line;
		win.draw(Circle_sens_line);
	}
	obstacle_point orient = rob_base.get_orientation();
	Circle_orient.setFillColor(sf::Color::Magenta);
	Circle_orient.setPosition(orient.x , orient.y);
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
	std::unique_lock<std::mutex> uLock(RobDrawMut);

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

