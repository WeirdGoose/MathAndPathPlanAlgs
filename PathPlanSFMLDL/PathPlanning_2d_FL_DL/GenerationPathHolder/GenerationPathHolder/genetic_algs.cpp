#include "gen_pars.h"
#include "genetic_algh.h"
#include "robot_AI.h"
#include "Whole_map.h"
#include "MapDraw.h"
#include <random>

std::random_device rd;
std::mt19937 gen(rd());

typedef struct rob_list_ {
	robot_params* rob_base_ptr;
	rob_pop_type_ identificator;

}rob_list;

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
	child_genes.copy_in_elements_to_arg(child_genes.oodsv);

	init_output_speed_terms(child_genes.osdsv);
	return child_genes;
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
	std::vector<_angle_type> orientation_angles;
	robs_positions.resize(rob_gen.size());
	orientation_angles.resize(rob_gen.size());

	start_position_rules(robs_positions, orientation_angles);

	for (rob_pop_type_ rob_num = 0; rob_num < rob_gen.size(); ++rob_num)
		rob_gen.at(rob_num).set_start_pos_and_dir(robs_positions.at(rob_num), orientation_angles.at(rob_num));
}

void genetic_start(std::vector<robot_params>& rob_gen, Whole_map &map, generation_type generation)
{
	// прим. большая часть исключений здесь для самопроверки
	if (!generation)
	{
		rob_pop_type_ identificator = 0;
		for (auto& rob_base : rob_gen)
		{
			genetic_set_pars(rob_base.genes, generation);
			init_logic(rob_base, rob_base.identificator, rob_base.genes);
		}
		return;
	}
	std::vector<robot_params*> sorted;
	for (int i = 0; i < rob_gen.size(); i++)
		sorted.push_back(&rob_gen.at(i));
	for (int i = 0; i < rob_gen.size(); i++)
	{
		for (int j = 0; j < sorted.size() - i; j++)
		{
			if (sorted.at(j)->steps_number > sorted.at(j + 1)->steps_number)
				std::swap(sorted.at(j)->steps_number, sorted.at(j + 1)->steps_number);
		}
	}

	static const size_t chosen_ones = GEN_POPULATION / 4;
	static const size_t new_ones = GEN_POPULATION / 4;
	static size_t one_parent_childs = 2 * (GEN_POPULATION - chosen_ones - new_ones) / chosen_ones;

	// проверка на то чтобы у каждых 2х родителей было одинаковое число потомков (кратность числа потомков родителям)
	if (!(GEN_POPULATION - chosen_ones - new_ones) % (chosen_ones / 2))
		throw std::exception("genetic_start::ERROR:: wrong population proportions");

	// выбирем первое четное число победителей
	if (chosen_ones % 2)
		throw std::exception("genetic_start::ERROR:: WRONG POPULATION NUMBER");

	for (int i = 0; i < chosen_ones; i++)
		sorted.at(i)->chosen_one = 1;

	// скрещиваем победителей, устанавливаем их потомков вместо проигравших, (по алгоритму должно сотаться несколько ячеек для новеньких)
	variables_set tmp_child_genes[4];
	for (rob_pop_type_ anoth_parent = 0; anoth_parent < chosen_ones; anoth_parent += 2)
	{
		if (sorted.at(anoth_parent)->chosen_one)
		{
			printf("parents %d %d cells : ", anoth_parent, anoth_parent + 1);
			for (rob_pop_type_ anoth_child_genes = 0; anoth_child_genes < one_parent_childs; anoth_child_genes++)
			{
				tmp_child_genes[anoth_child_genes] = genetic_crossing(sorted.at(anoth_parent)->genes, sorted.at(anoth_parent + 1)->genes);
				rob_pop_type_ child_replace_cell = (one_parent_childs + anoth_child_genes)*anoth_parent / 2;
				std::cout << child_replace_cell << " ";
				sorted.at(child_replace_cell)->genes.get_copy_from(tmp_child_genes[anoth_child_genes]);
				// ребенок тоже становиться chosen_one, чтобы определить оставшихся для новичков (типа мутации)
				if (sorted.at(child_replace_cell)->chosen_one)
					throw std::exception("genetic_start:: ERROR, WRONG CELL FOR CHILD");
				sorted.at(child_replace_cell)->chosen_one = 1;
			}
			std::cout << "\n";
		}
	}
	// заполняем новеньких на место оставшихся проигравших, не вытесненных потомками
	int check_cells_var = 0;		// переменная для самопроверки
	for (auto& rob_base : rob_gen)
	{
		if (!rob_base.chosen_one)
		{
			genetic_set_pars(rob_base.genes, generation);
			init_logic(rob_base, rob_base.identificator, rob_base.genes);
			check_cells_var++;
		}
	}
	if (check_cells_var != new_ones)
		throw std::exception("genetic_start:: ERROR, WRONG ACTUAL NUMBER OF NEW ONCE");
}
