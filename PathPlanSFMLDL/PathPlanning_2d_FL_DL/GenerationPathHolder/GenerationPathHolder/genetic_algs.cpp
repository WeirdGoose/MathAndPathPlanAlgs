#include "gen_pars.h"
#include "genetic_algh.h"
#include "robot_AI.h"
#include "Whole_map.h"
#include "MapDraw.h"
#include <random>

std::random_device rd;
std::mt19937 gen(rd());

float get_value_in_range(float left_boarder, float right_boarder)
{
	std::uniform_real_distribution<> dis(left_boarder, std::nextafter(right_boarder, DBL_MAX));
	return (float)dis(gen);
}


variables_set genetic_crossing(variables_set &genes_1, variables_set &genes_2)
{
	// thats a VERY bad practice, but well tHatS uNfORtUnaTe
#define LEFT_BOARDER(TERM_POINT) min(genes_1.TERM_POINT, genes_2.TERM_POINT)
#define RIGHT_BOARDER(TERM_POINT) max(genes_1.TERM_POINT, genes_2.TERM_POINT)

	variables_set child_genes;
	using std::min;
	using std::max;
	// peaks pars
	child_genes.iodsv.left_pars[0] = -90;
	child_genes.iodsv.left_pars[1] = -90;
	child_genes.iodsv.front_pars[1] = get_value_in_range(genes_1.iodsv.front_pars[1],
														genes_2.iodsv.front_pars[1]);
	// peaks of trapezoid close left
	child_genes.iodsv.leftp_pars[1] = get_value_in_range(max(child_genes.iodsv.left_pars[1], LEFT_BOARDER(iodsv.leftp_pars[1])),
														min(child_genes.iodsv.front_pars[1], RIGHT_BOARDER(iodsv.leftp_pars[1])));
	child_genes.iodsv.leftp_pars[2] = get_value_in_range(max(child_genes.iodsv.leftp_pars[1], LEFT_BOARDER(iodsv.leftp_pars[2])),
														min(child_genes.iodsv.front_pars[1], RIGHT_BOARDER(iodsv.leftp_pars[2])) );
	
	// right pars of trapezoid left
	child_genes.iodsv.left_pars[0] = get_value_in_range(max(child_genes.iodsv.leftp_pars[1], LEFT_BOARDER(iodsv.left_pars[0])),
														min(child_genes.iodsv.front_pars[1], RIGHT_BOARDER(iodsv.left_pars[0])));
	child_genes.iodsv.left_pars[3] = get_value_in_range(max(child_genes.iodsv.leftp_pars[2], LEFT_BOARDER(iodsv.left_pars[3])),
														min(child_genes.iodsv.front_pars[1], RIGHT_BOARDER(iodsv.left_pars[3])));
	
	// left and right pars of trapezoid close left
	child_genes.iodsv.leftp_pars[0] = get_value_in_range(max(child_genes.iodsv.leftp_pars[2], LEFT_BOARDER(iodsv.leftp_pars[0])),
														min(child_genes.iodsv.front_pars[3], RIGHT_BOARDER(iodsv.leftp_pars[0])));
	child_genes.iodsv.leftp_pars[3] = get_value_in_range(max(child_genes.iodsv.leftp_pars[2], LEFT_BOARDER(iodsv.leftp_pars[3])),
														min(child_genes.iodsv.front_pars[1], RIGHT_BOARDER(iodsv.leftp_pars[3])));

	// left and right pars of trapezoid center
	child_genes.iodsv.front_pars[0] = get_value_in_range(max(child_genes.iodsv.leftp_pars[2], LEFT_BOARDER(iodsv.front_pars[0])),
														min(child_genes.iodsv.leftp_pars[3], RIGHT_BOARDER(iodsv.front_pars[0])));
	child_genes.add_right_elems_symm(child_genes.iodsv);

	init_output_speed_terms(child_genes.osdsv);
}

void genetic_set_pars(variables_set &genes, generation_type &generation)
{
	const float acc_bia = 5;	
	
	// peaks pars
	genes.iodsv.left_pars[0] = -90;
	genes.iodsv.left_pars[1] = -90;
	genes.iodsv.front_pars[1] = get_value_in_range(-acc_bia, 0);

	// peaks of trapezoid close left
	genes.iodsv.leftp_pars[1] = get_value_in_range(genes.iodsv.left_pars[1], 
													genes.iodsv.front_pars[1]);
	genes.iodsv.leftp_pars[2] = get_value_in_range(genes.iodsv.leftp_pars[1], 
													genes.iodsv.front_pars[1]);

	// right pars of trapezoid left
	genes.iodsv.left_pars[2] = get_value_in_range(genes.iodsv.left_pars[1], 
													genes.iodsv.leftp_pars[1]);
	genes.iodsv.left_pars[3] = get_value_in_range(genes.iodsv.left_pars[2], 
													genes.iodsv.leftp_pars[1]);

	// left and right pars of trapezoid close left
	genes.iodsv.leftp_pars[0] = get_value_in_range(genes.iodsv.left_pars[2], 
													genes.iodsv.left_pars[3]);
	genes.iodsv.leftp_pars[3] = get_value_in_range(genes.iodsv.leftp_pars[2], 
													genes.iodsv.front_pars[1]);

	// left and right pars of trapezoid center
	genes.iodsv.front_pars[0] = get_value_in_range(genes.iodsv.leftp_pars[2], 
													genes.iodsv.leftp_pars[3]);
	genes.add_right_elems_symm(genes.iodsv);

	// copies vectors (output the same as input, correction by rules)
	genes.copy_in_elements_to_arg(genes.oodsv);

	init_output_speed_terms(genes.osdsv);

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
	for (auto& rob_base : rob_gen)
	{
		variables_set genes;
		genetic_set_pars(genes, generation);
		init_logic(rob_base, identificator++, genes);
	}
}