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
std::mutex synchMutex2;
std::condition_variable synchCndVar;


// TODO
/*
	check_sensors придётся вынести в main()
	rob_base.position и rob_base.aim повторить в map, и роббэйзовые сделать в логике а в мэпе настоящие, сделать согласование
	выбирать направление из чека сенсоров и менять скорость по выбранному напрвлению за один шаг цикла
	добавить изменение скорости и перемещения в 0 при столкновении с препятствием
*/

std::chrono::steady_clock::time_point time_begin;
std::chrono::steady_clock::time_point time_end;

int main() 
{
	unsigned int map_width = MAX_MAP_SIZE_X;
	unsigned int map_heigth = MAX_MAP_SIZE_Y;
	simulation sim1;
	std::vector<robot_params> rob_gen;
	rob_gen.resize(GEN_POPULATION);

	Whole_map map(map_width, map_heigth); 
	//Create
	std::vector<std::thread*> rob_gen_ai;
	rob_gen_ai.resize(GEN_POPULATION);

	set_map(map);
	std::thread graphics(scene_movment, std::ref(map), std::ref(rob_gen), std::ref(sim1));

	for (generation_type gen_number = 0; gen_number < GENERATIONS_NUM; gen_number++)
	{
		sim1.off_simulation();
		genetic_init(rob_gen);
		genetic_start(rob_gen, map, gen_number);

		time_begin = std::chrono::steady_clock::now();

		for (rob_pop_type_ rob_num = 0; rob_num < rob_gen.size(); ++rob_num)
		{
			std::thread * new_thread = new std::thread(robot_logic, std::ref(map), std::ref(rob_gen.at(rob_num)), std::ref(sim1));
			rob_gen_ai.push_back(new_thread);
		}
		sim1.start_simulation();

		std::unique_lock<std::mutex> uLock(synchMutex2);
		synchCndVar.wait(uLock);
		for (rob_pop_type_ rob_num = 0; rob_num < rob_gen.size(); ++rob_num)
			delete rob_gen_ai.at(rob_num);
	}
	

	while (sim1.simulation_state()) 
		Sleep(100);

	char c;
	cin >> c;
	return 1;
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
			//else
				//cout << "WARNING WRONG SENSORS CALCULATE!!!\n";
			
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
BOOL make_one_step(Whole_map &map, robot_params &rob_base)
{
	rob_pop_type_ identificator = rob_base.identificator;
	_speed_type real_speed = rob_base.get_speed() + ROB_ERROR_SPEED;
	map.orientation_angle.at(identificator) = rob_base.orientation_angle + ROB_ERROR_ROTATION;
	
	if (map.at(map.robs_positions.at(identificator).x, map.robs_positions.at(identificator).y) == OBSTACLE_MAP_CHAR)
	{
		real_speed = 0;
		rob_base.failure = 1;
		rob_base.steps_number = LLONG_MAX - rob_base.steps_number;
		return 1;
	}
	map.robs_positions.at(identificator).x = rob_base.position.x + real_speed * cos(map.orientation_angle.at(identificator)) * rob_base.delta_t;
	map.robs_positions.at(identificator).y = rob_base.position.y + real_speed * sin(map.orientation_angle.at(identificator)) * rob_base.delta_t;
	rob_base.position.x = map.robs_positions.at(identificator).x;
	rob_base.position.y = map.robs_positions.at(identificator).y;
	rob_base.orientation_angle = map.orientation_angle.at(identificator);
	rob_base.path.push_back(rob_base.position);
	rob_base.set_speed(real_speed);
	rob_base.steps_number++;
}

void signal_to_draw_if_i_last(){
	static uint32_t ended_robots = 0;
	if (ended_robots == GEN_POPULATION - 1)
	{
		time_end = std::chrono::steady_clock::now();
		std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_begin).count() << "\t[us]" << std::endl;
		std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::nanoseconds> (time_end - time_begin).count() << "\t[ns]" << std::endl;

		SIGNAL_TO_DRAW(synchCndVar);
		ended_robots = 0;
	}
	else 
		ended_robots++;
}

void robot_logic(Whole_map &map, robot_params &rob_base, simulation &sim)
{
	std::unique_lock<std::mutex> uLock(synchMutex);
	sensor_point *sensor_points_ptr; 
	BOOL exit_ctrl = 0;
	sensor_points_ptr = rob_base.get_sensor_points();

	while (sim.simulation_state()) 
	{
		//synchCndVar.wait(uLock);
		WAIT_FOR_DRAW(synchCndVar, uLock);
		if (exit_ctrl)
		{
			signal_to_draw_if_i_last();
			return;
		}
		check_sensors(map, rob_base);
		exit_ctrl = robot_active_cyc(map, rob_base, _fuzzy_set_);
		sensor_points_ptr = rob_base.get_sensor_points();
		
		
		exit_ctrl = make_one_step(map, rob_base);
		direct_sensors(sensor_points_ptr, rob_base.orientation_angle, rob_base.position);
		rob_base.steps_number++;
		//sensor_points_ptr = rob_base.get_sensor_points();
		//direct_sensors(sensor_points_ptr, map.orientation_angle, map.rob_position);
	}
}

