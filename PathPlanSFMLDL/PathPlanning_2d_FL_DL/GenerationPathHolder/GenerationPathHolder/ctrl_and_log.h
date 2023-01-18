#pragma once

#include <iostream>
#include <list>    
#include <SFML/Graphics.hpp>
#include <thread>
#include <mutex>
#include <windows.h>
#include "gen_pars.h"
#include <synchapi.h>
#include <condition_variable>
#include <fstream>

extern class Whole_map;
extern class robot_params;
extern class simulation;

void debug_term_cmd(Whole_map& map, std::vector<robot_params>& rob_gen, simulation& sim1);
void write_log(const std::string &text, const std::string &filename = "gen_log.txt");