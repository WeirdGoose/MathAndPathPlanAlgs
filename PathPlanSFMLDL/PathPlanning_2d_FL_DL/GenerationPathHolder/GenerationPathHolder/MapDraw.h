#pragma once

#include <iostream>
#include <list>    
#include <SFML/Graphics.hpp>
#include "Whole_map.h"
#include "robot_AI.h"
#include "gen_pars.h"

// header of the main programm
#define AIM_RADIUS	6


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
void draw_map(	sf::RenderWindow &win,
				Whole_map &map,
				robot_params &rob_base,
				std::vector<sf::CircleShape> &obs_space,
				std::vector<sf::CircleShape> &sens_line_space,
				std::vector<sf::CircleShape> &path_space);

void scene_movment(	Whole_map &map, 
					robot_params &rob_base, 
					simulation &sim	);
void robot_logic(Whole_map &map, 
					robot_params &rob_base,
					simulation &sim);
