#include <iostream>
#include <list>    
#include <SFML/Graphics.hpp>
#include "MapDraw.h"
#include <thread>
#include <mutex>
#include <windows.h>
#include "gen_pars.h"
#include <synchapi.h>
#include <condition_variable>
#include <fstream>
#include "ctrl_and_log.h"

extern std::condition_variable synchCndVar;


void debug_term_cmd(Whole_map& map, std::vector<robot_params>& rob_gen, simulation& sim1)
{
	std::string cmd_str;
	int index = 0;
	std::string nl_str = "nl";
	std::string draw_str = "draw";
	std::string help_str = "help";

	if (!sim1.simulation_state())
		while (!sim1.simulation_state()) Sleep(1);
	while (sim1.simulation_state())
	{
		std::cin >> cmd_str;
		if (!strcmp(cmd_str.c_str(), draw_str.c_str()))
			synchCndVar.notify_one();
		else if (cmd_str.find(nl_str, index) == 0)
			draw_obs_vars(rob_gen.at(std::atoi(cmd_str.substr(nl_str.size(), cmd_str.size() - nl_str.size()).c_str())).genes.iodsv);
		else if (!strcmp(cmd_str.c_str(), help_str.c_str()))
			std::cout << "type \"" << draw_str << "\" to draw current bots position \n"
			<< "type \"" << nl_str << "[rob identificator]\" to to get fuzzy variables \n";
	}
}

void write_log(const std::string &text, const std::string &filename)
{
	std::ofstream log_file(
		filename, std::ios_base::out | std::ios_base::app);
	log_file << text << std::endl;
}