#include "gen_pars.h"
#include "genetic_algh.h"
#include "robot_AI.h"
#include "Whole_map.h"
#include "MapDraw.h"
#include <random>

float get_value_in_range(float left_boarder, float right_boarder, std::mt19937& gen)
{
	std::uniform_real_distribution<> dis(left_boarder, std::nextafter(right_boarder, DBL_MAX));
	return (float)dis(gen);
}

void genetic_recalculate_pars(variables_set &genes, generation_type &generation)
{

}

void genetic_set_pars(variables_set &genes, generation_type &generation)
{
	const float acc_bia = 5;	
	std::random_device rd;
	std::mt19937 gen(rd());

	genes.iodsv.left_pars.resize(4);
	genes.iodsv.leftp_pars.resize(4);
	genes.iodsv.front_pars.resize(4);
	genes.iodsv.rightp_pars.resize(4);
	genes.iodsv.right_pars.resize(4);

	// peaks pars
	genes.iodsv.left_pars[0] = -90;
	genes.iodsv.left_pars[1] = -90;
	genes.iodsv.front_pars[1] = -5;
	genes.iodsv.front_pars[2] = 5;
	genes.iodsv.right_pars[2] = 90;
	genes.iodsv.right_pars[3] = 90;

	// peaks of trapezoid close right
	genes.iodsv.rightp_pars[1] = get_value_in_range(genes.iodsv.front_pars[2] + acc_bia, genes.iodsv.right_pars[1] - acc_bia, gen);
	genes.iodsv.rightp_pars[2] = get_value_in_range(genes.iodsv.rightp_pars[1] + acc_bia, genes.iodsv.right_pars[1]  - acc_bia, gen);

	// peaks of trapezoid close left
	genes.iodsv.leftp_pars[1] = get_value_in_range(genes.iodsv.left_pars[1] + acc_bia, genes.iodsv.front_pars[1] - acc_bia, gen);
	genes.iodsv.leftp_pars[2] = get_value_in_range(genes.iodsv.leftp_pars[1] + acc_bia, genes.iodsv.front_pars[1] - acc_bia, gen);

	// right pars of trapezoid left
	genes.iodsv.left_pars[2] = get_value_in_range(genes.iodsv.left_pars[1], genes.iodsv.leftp_pars[1],  gen);
	genes.iodsv.left_pars[3] = genes.iodsv.leftp_pars[1];

	// left pars of trapezoid right
	genes.iodsv.right_pars[1] = get_value_in_range(genes.iodsv.rightp_pars[2], genes.iodsv.right_pars[2], gen);
	genes.iodsv.right_pars[0] = genes.iodsv.rightp_pars[2];

	// left and right pars of trapezoid close left
	genes.iodsv.leftp_pars[0] = genes.iodsv.left_pars[2];
	genes.iodsv.leftp_pars[3] = genes.iodsv.front_pars[1];

	// left and right pars of trapezoid close right
	genes.iodsv.rightp_pars[0] = genes.iodsv.front_pars[2];
	genes.iodsv.rightp_pars[3] = genes.iodsv.right_pars[1];

	// left and right pars of trapezoid center
	genes.iodsv.front_pars[0] = genes.iodsv.leftp_pars[2];
	genes.iodsv.front_pars[3] = genes.iodsv.rightp_pars[1];

	// copies vectors (output the same as input, correction by rules)
	genes.oodsv.left_pars	= genes.iodsv.left_pars;
	genes.oodsv.leftp_pars	= genes.iodsv.leftp_pars;
	genes.oodsv.front_pars	= genes.iodsv.front_pars;
	genes.oodsv.rightp_pars = genes.iodsv.rightp_pars;
	genes.oodsv.right_pars	= genes.iodsv.right_pars;

	init_input_speed_terms();
}

void genetic_init(std::vector<robot_params>& rob_gen)
{
	if(rob_gen.size() != GEN_POPULATION)
		rob_gen.resize(GEN_POPULATION);
	std::vector<obstacle_point> robs_positions;
	robs_positions.resize(rob_gen.size());

	start_position_rules(robs_positions);
	for (rob_pop_type_ rob_num = 0; rob_num < rob_gen.size(); ++rob_num)
		rob_gen.at(rob_num).set_start_pos_and_dir(robs_positions.at(rob_num), START_ANGLE);

}

void genetic_start(std::vector<robot_params>& rob_gen, Whole_map &map, generation_type &generation)
{
	rob_pop_type_ identificator = 0;
	variables_set genes;
	for (auto& rob_base : rob_gen)
	{
		genetic_set_pars(genes, generation);
		init_logic(rob_base, identificator++, genes);
	}
}