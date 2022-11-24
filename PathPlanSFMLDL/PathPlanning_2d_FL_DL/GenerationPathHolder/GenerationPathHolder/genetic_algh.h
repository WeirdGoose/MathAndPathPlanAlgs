#pragma once
#include "gen_pars.h"
#include "robot_AI.h"
#include "Whole_map.h"
#include "MapDraw.h"
#include "manu_fuzzy_logic.h" 



void genetic_init(std::vector<robot_params>& rob_gen);
void genetic_start(std::vector<robot_params>& rob_gen, Whole_map &map, uint8_t generation);
