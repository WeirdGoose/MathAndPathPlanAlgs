#pragma once

#include <iostream>
#include <list>    
#include <SFML/Graphics.hpp>
#include "Whole_map.h"
#include "robot_AI.h"
#include "gen_pars.h"

// header of the main programm

#define DRAW_AFTER_SIM_MODE 0
#define ROBS_DIFF_POS 0
#define DRAW_WITH_POINTS 0

#if DRAW_AFTER_SIM_MODE 
#define SIGNAL_TO_DRAW(SYNCH)		SYNCH.notify_one()
#define WAIT_FOR_DRIVE(SYNCH, LOCK)	SYNCH.wait(LOCK)
#define SIGNAL_TO_DRIVE(SYNCH)		do{}while(0)
#define WAIT_FOR_DRAW(SYNCH, LOCK)	do{}while(0)

#else
#define SIGNAL_TO_DRIVE(SYNCH)		SYNCH.notify_one();
#define WAIT_FOR_DRAW(SYNCH, LOCK)	SYNCH.wait(LOCK)
#define SIGNAL_TO_DRAW(SYNCH)		do{}while(0)
#define WAIT_FOR_DRIVE(SYNCH, LOCK)	do{}while(0)

#endif



class simulation {
	bool simulation_run = 0;
public:
	bool simulation_state() {
		return this->simulation_run;
	}
	void off_simulation() {
		this->simulation_run = 0;
	}
	void start_simulation() {
		this->simulation_run = 1;
	}
};

void set_map(Whole_map &map);

void draw_map(sf::RenderWindow &win,
	Whole_map &map,
	std::vector<sf::CircleShape> &obs_space);

void draw_other(sf::RenderWindow &win,
	Whole_map &map,
	robot_params &rob_base,
	std::vector<sf::CircleShape> &sens_line_space);

void start_position_rules(std::vector<obstacle_point> &robs_positions, std::vector<_angle_type>& orientation_angle);
void scene_movment(	Whole_map &map, 
					std::vector<robot_params>& rob_gen,
					simulation &sim	);
void direct_sensors(sensor_point *sensor_points, _angle_type angle, obstacle_point rob_pos);
void robot_logic(Whole_map &map, 
					robot_params &rob_base,
					simulation &sim);
