#pragma once

#include "gen_pars.h"
#include "fl/Headers.h"

#define SPEED_TERMS_NUM 3
#define ANGLE_TERMS_NUM 3
#define TRIA_TERMS_SPACE_SIZE 50
#define SPEED_ACCESSORY_MAX 1 

typedef float _speed_space;


void init_fuzzy(fl::Engine* engine,
	fl::InputVariable* speed,
	fl::InputVariable* obs_angle,
	fl::OutputVariable* mSteer,
	fl::OutputVariable* outSpeed,
	fl::RuleBlock* mamdani);
