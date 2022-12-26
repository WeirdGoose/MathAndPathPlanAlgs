#pragma once
#include "gen_pars.h"
#include "robot_AI.h"
#include "Whole_map.h"
#include "MapDraw.h"
#include "manu_fuzzy_logic.h" 

#define GENERATIONS_NUM 10
#define GEN_POPULATION 24

typedef uint8_t generation_type;

void genetic_init(std::vector<robot_params>& rob_gen);
void genetic_start(std::vector<robot_params>& rob_gen, Whole_map &map, generation_type generation);