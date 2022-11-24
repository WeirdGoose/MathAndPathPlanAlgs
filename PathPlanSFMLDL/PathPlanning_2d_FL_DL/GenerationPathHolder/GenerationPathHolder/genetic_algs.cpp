#include "gen_pars.h"
#include "genetic_algh.h"
#include "robot_AI.h"
#include "Whole_map.h"
#include "MapDraw.h"

void genetic_set_pars(variables_set &genes)
{

}

void genetic_init(std::vector<robot_params>& rob_gen)
{
	rob_gen.resize(GEN_POPULATION);
	std::vector<obstacle_point> robs_positions;
	robs_positions.resize(rob_gen.size());

	start_position_rules(robs_positions);
	for (rob_pop_type_ rob_num = 0; rob_num < rob_gen.size(); ++rob_num)
		rob_gen.at(rob_num).set_start_pos_and_dir(robs_positions.at(rob_num), START_ANGLE);

}

void genetic_start(std::vector<robot_params>& rob_gen, Whole_map &map, uint8_t generation)
{
	if (generation == 0)
	{
		rob_pop_type_ identificator = 0;
		for (auto& rob_base : rob_gen)
			init_logic(rob_base, identificator++);
	}
}