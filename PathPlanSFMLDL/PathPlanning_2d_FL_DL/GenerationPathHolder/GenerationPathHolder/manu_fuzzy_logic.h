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

class variables_set {
public:
	obs_desition_space		iodsv; // input obstacle desition vector
	speed_desition_space	isdsv; // input speed desition vector
	obs_desition_space		oodsv; // output obstacle desition vector
	speed_desition_space	osdsv; // output speed desition vector
	variables_set() {
		this->iodsv.left_pars.resize(4);
		this->iodsv.leftp_pars.resize(4);
		this->iodsv.front_pars.resize(4);
		this->iodsv.rightp_pars.resize(4);
		this->iodsv.right_pars.resize(4);

		this->oodsv.left_pars.resize(4);
		this->oodsv.leftp_pars.resize(4);
		this->oodsv.front_pars.resize(4);
		this->oodsv.rightp_pars.resize(4);
		this->oodsv.right_pars.resize(4);

		this->isdsv.low_pars.resize(4);
		this->isdsv.middle_pars.resize(4);
		this->isdsv.high_pars.resize(4);

		this->osdsv.low_pars.resize(4);
		this->osdsv.middle_pars.resize(4);
		this->osdsv.high_pars.resize(4);
	}
	void copy_in_elements_to_arg(obs_desition_space &dsv)
	{
		dsv.left_pars =	this->iodsv.left_pars;
		dsv.leftp_pars = this->iodsv.leftp_pars;
		dsv.front_pars = this->iodsv.front_pars;
		dsv.rightp_pars = this->iodsv.rightp_pars;
		dsv.right_pars = this->iodsv.right_pars;
	}
	void copy_in_elements_to_arg(speed_desition_space &dsv)
	{
		dsv.low_pars =	this->isdsv.low_pars;
		dsv.middle_pars = this->isdsv.middle_pars;
		dsv.high_pars = this->isdsv.high_pars;
	}
	void copy_out_elements_to_arg(obs_desition_space &dsv)
	{
		dsv.left_pars = this->oodsv.left_pars;
		dsv.leftp_pars = this->oodsv.leftp_pars;
		dsv.front_pars = this->oodsv.front_pars;
		dsv.rightp_pars = this->oodsv.rightp_pars;
		dsv.right_pars = this->oodsv.right_pars;
	}
	void copy_out_elements_to_arg(speed_desition_space &dsv)
	{
		dsv.low_pars = this->osdsv.low_pars;
		dsv.middle_pars = this->osdsv.middle_pars;
		dsv.high_pars = this->osdsv.high_pars;
	}
	void add_right_elems_symm(obs_desition_space &dsv)
	{
		for (uint8_t idx = 0; idx < dsv.left_pars.size(); idx++)
			dsv.right_pars[idx] = -dsv.left_pars[dsv.left_pars.size() - idx - 1];

		for (uint8_t idx = 0; idx < dsv.leftp_pars.size(); idx++)
			dsv.rightp_pars[idx] = -dsv.leftp_pars[dsv.leftp_pars.size() - idx - 1];

		for (uint8_t idx = 0; idx < dsv.front_pars.size()/2; idx++)
			dsv.front_pars[idx] = -dsv.front_pars[dsv.front_pars.size() - idx - 1];
	}

};

void init_fuzzy(fl::Engine* engine,
	fl::InputVariable* speed,
	fl::InputVariable* obs_angle,
	fl::OutputVariable* mSteer,
	fl::OutputVariable* outSpeed,
	fl::RuleBlock* mamdani);

void init_fuzzy_ext(fl::Engine* engine,
	fl::InputVariable* speed,
	fl::InputVariable* obs_angle,
	fl::OutputVariable* mSteer,
	fl::OutputVariable* outSpeed,
	fl::RuleBlock* mamdani,
	variables_set	&var_s);

void init_input_speed_terms(speed_desition_space &dsv);
void init_output_speed_terms(speed_desition_space &dsv);