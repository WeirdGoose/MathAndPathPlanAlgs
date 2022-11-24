#pragma once

#include "gen_pars.h"
#include "fl/Headers.h"

#define SPEED_TERMS_NUM 3
#define ANGLE_TERMS_NUM 3
#define TRIA_TERMS_SPACE_SIZE 50
#define SPEED_ACCESSORY_MAX 1 

typedef float _speed_space;

typedef struct obs_desition_space_ {
	std::vector<float> left_pars;
	std::vector<float> leftp_pars;
	std::vector<float> front_pars;
	std::vector<float> rightp_pars;
	std::vector<float> right_pars;
}obs_desition_space;

typedef struct speed_desition_space_ {
	std::vector<float> low_pars;
	std::vector<float> middle_pars;
	std::vector<float> high_pars;
}speed_desition_space;

typedef struct variables_set_ {
	obs_desition_space		iodsv; // input obstacle desition vector
	speed_desition_space	isdsv; // input speed desition vector
	obs_desition_space		oodsv; // output obstacle desition vector
	speed_desition_space	osdsv; // output speed desition vector
}variables_set;

void init_fuzzy(fl::Engine* engine,
	fl::InputVariable* speed,
	fl::InputVariable* obs_angle,
	fl::OutputVariable* mSteer,
	fl::OutputVariable* outSpeed,
	fl::RuleBlock* mamdani);
