#include <iostream>
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


HANDLE sleepTimerMutex;
std::mutex synchMutex;
std::condition_variable synchCndVar;
/*
void sleep_accurate(uint32_t msec)
{
	__int64 qwDueTime = msec * 
	SetWaitableTimer();
}
*/

// TODO
/*
	check_sensors придётся вынести в main()
	rob_base.position и rob_base.aim повторить в map, и роббэйзовые сделать в логике а в мэпе настоящие, сделать согласование
	выбирать направление из чека сенсоров и менять скорость по выбранному напрвлению за один шаг цикла
	добавить изменение скорости и перемещения в 0 при столкновении с препятствием
*/


int main() 
{
	unsigned int map_width = MAX_MAP_SIZE_X;
	unsigned int map_heigth = MAX_MAP_SIZE_Y;
	simulation sim1;
	//robot_params rob_base;
	std::vector<robot_params> rob_gen;
	rob_gen.resize(GEN_POPULATION);

	Whole_map map(map_width, map_heigth); 
	//Create
	genetic_init(rob_gen);
	set_map(map);

	genetic_start(rob_gen, map, 0);

	sim1.start_simulation();
	std::vector<std::thread*> rob_gen_ai;
	rob_gen_ai.resize(GEN_POPULATION);
	for (rob_pop_type_ rob_num = 0; rob_num < rob_gen.size(); ++rob_num)
	{
		std::thread * new_thread = new std::thread(robot_logic, std::ref(map), std::ref(rob_gen.at(rob_num)), std::ref(sim1));
		rob_gen_ai.push_back(new_thread);
	}

	//robot_ai.join();
	std::thread graphics(scene_movment, std::ref(map), std::ref(rob_gen), std::ref(sim1));
	//graphics.join();
	
	while (sim1.simulation_state()) 
	{
		Sleep(100);
	}

	for (rob_pop_type_ rob_num = 0; rob_num < rob_gen.size(); ++rob_num)
	{
		delete rob_gen_ai.at(rob_num);
	}

	char c;
	cin >> c;
	return 1;
}

void start_position_rules(std::vector<obstacle_point> &robs_positions)
{
	rob_pop_type_ idx=0;
	robs_positions.resize(GEN_POPULATION);
	for (auto &position: robs_positions)
	{
		position.x = START_ROBOT_POS_X + (idx++)*5;
		position.y = START_ROBOT_POS_Y;
	}
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

	start_position_rules(map.robs_positions);
}

void draw_map(	sf::RenderWindow &win, 
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

void scene_movment(Whole_map &map, std::vector<robot_params>& rob_gen, simulation &sim)
{

	sf::RenderWindow window(sf::VideoMode(map.get("width"), map.get("height")), "SFML works!");
	std::vector<sf::CircleShape> Obs;
	std::vector<map_point> Path;
	std::vector<sf::CircleShape> lines;
	unsigned long size = map.obstacles_weight;
	map_point robot_pos;

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

		window.clear();
		draw_map(window, map, Obs);
		for (auto& rob_base : rob_gen)
		{
			robot_pos.x = rob_base.position.x;
			robot_pos.y = rob_base.position.y;
			if (map.at(robot_pos.x, robot_pos.y) != ROBOT_PATH_MAP_CHAR) 
			{
				// save to map last position
				Path.push_back(robot_pos);
				map.at(robot_pos.x, robot_pos.y) = ROBOT_PATH_MAP_CHAR;
			}
		}
		for (auto& rob_base : rob_gen)
			draw_other(window, map, rob_base, lines, Path);
		window.display();
		synchCndVar.notify_one();
		//Sleep(1);
	}
	sim.off_simulation();
}


// checked is there are obstacle by compare points of every sensor line with obstacle coordinates
void check_sensors(Whole_map &map, robot_params &rob_base)
{
	sensor_point *sensor_points_ptr;
	sensor_points_ptr = rob_base.get_sensor_points();
	sensor_point sens_obs;
	map_point sens_disc;
	map_point rob_pos_disc;
	
	for (_sensor_num_type i = 0; i < rob_base.rob_lines_num; ++i)
	{
		sens_obs = sensor_points_ptr[i];
		rob_base.sensors_trigg(i, LINES_RADIUS + 1, EMPTY_SPACE_MAP_CHAR);
		for (int8_t j = 0; j < rob_base.sens_math_lambdas.size(); ++j)
		{
			sens_disc.x = (rob_base.position.x + rob_base.sens_math_lambdas[j] * sens_obs.pos.x) / (1 + rob_base.sens_math_lambdas[j]);
			sens_disc.y = (rob_base.position.y + rob_base.sens_math_lambdas[j] * sens_obs.pos.y) / (1 + rob_base.sens_math_lambdas[j]);
			if (sens_disc.x < MAX_MAP_SIZE_X && sens_disc.y < MAX_MAP_SIZE_Y)
			{
				if (map.at(sens_disc.x, sens_disc.y) == OBSTACLE_MAP_CHAR)
				{
					rob_base.sensors_trigg(i, j, OBSTACLE_MAP_CHAR);
					break;
				}
			}
			else
				cout << "WARNING WRONG SENSORS CALCULATE!!!\n";
			
		}
	}
	// return the pointer to sensor_array
}

void direct_sensors(sensor_point *sensor_points, _angle_type angle, obstacle_point rob_pos)
{
	//cout << "---------------sensor lines coodrinates redir--------------- \n\n\n";
	// right

	for (_sensor_num_type i = 0; i < LINES_NUMBER / 2 + 1; ++i)
	{
		sensor_points[i].pos.x = rob_pos.x + LINES_RADIUS * cos(SENSOR_RAD_STEP * i + angle);
		sensor_points[i].pos.y = rob_pos.y + LINES_RADIUS * sin(SENSOR_RAD_STEP * i + angle);
		sensor_points[i].state = EMPTY_SPACE_MAP_CHAR;
		sensor_points[i].distant = 0;
		//cout << "x - " << this->sensor_points[i].pos.x << "\t| y - " << this->sensor_points[i].pos.y << "\n-----------------\n";
	}

	// left
	for (_sensor_num_type i = LINES_NUMBER / 2 + 1; i < LINES_NUMBER; ++i)
	{
		sensor_points[i].pos.x = rob_pos.x + LINES_RADIUS * cos(-SENSOR_RAD_STEP * (i - LINES_NUMBER / 2) + angle);
		sensor_points[i].pos.y = rob_pos.y + LINES_RADIUS * sin(-SENSOR_RAD_STEP * (i - LINES_NUMBER / 2) + angle);
		sensor_points[i].state = EMPTY_SPACE_MAP_CHAR;
		sensor_points[i].distant = 0;
		//cout << "x - " << this->sensor_points[i].pos.x << "\t| y - " << this->sensor_points[i].pos.y << "\n-----------------\n";
	}
}

// moves through the map, depending on speed, orientation and delta t
void make_one_step(Whole_map &map, robot_params &rob_base)
{
	_speed_type real_speed = rob_base.get_speed() + ROB_ERROR_SPEED;
	map.orientation_angle = rob_base.orientation_angle + ROB_ERROR_ROTATION;
	rob_pop_type_ identificator = rob_base.identificator;
	if (map.at(map.robs_positions.at(identificator).x, map.robs_positions.at(identificator).y) == OBSTACLE_MAP_CHAR)
	{
		real_speed = 0;
	}
	map.robs_positions.at(identificator).x = rob_base.position.x + real_speed * cos(map.orientation_angle) * rob_base.delta_t;
	map.robs_positions.at(identificator).y = rob_base.position.y + real_speed * sin(map.orientation_angle) * rob_base.delta_t;
	rob_base.position.x = map.robs_positions.at(identificator).x;
	rob_base.position.y = map.robs_positions.at(identificator).y;
	rob_base.orientation_angle = map.orientation_angle;
	
	rob_base.set_speed(real_speed);
}

void robot_logic(Whole_map &map, robot_params &rob_base, simulation &sim)
{
	std::unique_lock<std::mutex> uLock(synchMutex);
	sensor_point *sensor_points_ptr; 
	BOOL exit_ctrl = 0;
	sensor_points_ptr = rob_base.get_sensor_points();
	while (sim.simulation_state()) 
	{
		synchCndVar.wait(uLock);
		if (exit_ctrl)
		{
			delete rob_base.engine;
			delete rob_base.fl_speed;
			delete rob_base.fl_obs_angle;
			delete rob_base.mSteer;
			delete rob_base.fl_outSpeed;
			delete rob_base.mamdani;
			return;
		}
		check_sensors(map, rob_base);
		exit_ctrl = robot_active_cyc(map, rob_base, _fuzzy_set_);
		sensor_points_ptr = rob_base.get_sensor_points();
		direct_sensors(sensor_points_ptr, rob_base.orientation_angle, rob_base.position);
		
		check_sensors(map, rob_base);
		exit_ctrl = robot_active_cyc(map, rob_base, _speed_setting_);

		make_one_step(map, rob_base);
		//sensor_points_ptr = rob_base.get_sensor_points();
		//direct_sensors(sensor_points_ptr, map.orientation_angle, map.rob_position);
	}
}
