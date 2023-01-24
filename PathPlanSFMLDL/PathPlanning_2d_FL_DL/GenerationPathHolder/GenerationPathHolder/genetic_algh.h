#pragma once
#include "gen_pars.h"
#include "robot_AI.h"
#include "Whole_map.h"
#include "MapDraw.h"
#include "manu_fuzzy_logic.h" 

#define GENERATIONS_NUM 40
#define GEN_POPULATION	32

typedef uint8_t generation_type;

class genetic_log
{
public:
	rob_pop_type_ identificator;
	variables_set genes;
	std::vector<obstacle_point> path;
	unsigned long long fine;
	BOOL failure;
	generation_type generation;
	genetic_log()
	{
		this->failure = 0;
	}
	void save_rob_info(generation_type generation, 
		variables_set& genes, 
		std::vector<obstacle_point>& path,
		rob_pop_type_ identificator,
		BOOL failure)
	{
		this->path = path;
		this->identificator = identificator;
		this->failure = failure;
		this->genes.get_copy_from(genes);
		this->generation = generation;
	}
	void stop_and_output_log() 
	{
		printf("generation's %d rob (id %d), path size %d, failure is %d", 
			this->generation, this->identificator, this->path.size(), this->failure);

		draw_speed_vars(this->genes.isdsv);
		draw_obs_vars(this->genes.iodsv);

		cout << "pres any key to go out from pic, and continue\n";
		std::string str;
		cin >> str;
	}
};


void genetic_init(std::vector<robot_params>& rob_gen);

void genetic_start(std::vector<robot_params>& rob_gen,
	Whole_map &map,
	generation_type generation,
	std::vector<genetic_log>& gen_logger);
