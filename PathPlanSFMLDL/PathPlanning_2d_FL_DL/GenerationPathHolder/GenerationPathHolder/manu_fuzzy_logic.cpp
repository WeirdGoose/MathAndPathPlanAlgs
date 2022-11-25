#include "manu_fuzzy_logic.h"
#include "fl/Headers.h"
#include "robot_AI.h"


void init_input_obs_terms(obs_desition_space &dsv)
{

	std::vector<float> left_pars	{-90, -80, -70, -60};
	std::vector<float> leftp_pars	{-70, -50, -30, -5};
	std::vector<float> front_pars	{-10, -5, 5, 10};
	std::vector<float> rightp_pars	{5, 30, 50, 70};
	std::vector<float> right_pars	{60, 70, 80, 90};

	dsv.left_pars = left_pars;
	dsv.leftp_pars = leftp_pars;
	dsv.front_pars = front_pars;
	dsv.rightp_pars = rightp_pars;
	dsv.right_pars = right_pars;

}

void init_input_speed_terms(speed_desition_space &dsv)
{
	std::vector<float> low_pars		{0, 0.5, 1, 1.5};
	std::vector<float> middle_pars	{1, 2, 2.5, 3};
	std::vector<float> high_pars	{2.5, 4, 5, 7};
	dsv.low_pars = low_pars;
	dsv.middle_pars = middle_pars;
	dsv.high_pars = high_pars;
}

void init_output_obs_terms(obs_desition_space &dsv)
{
	std::vector<float> left_pars	{ -90, -90, -90, -60 };
	std::vector<float> leftp_pars	{ -70, -50, -30, -5 };
	std::vector<float> front_pars	{ -10, -5, 5, 10 };
	std::vector<float> rightp_pars	{ 5, 30, 50, 70 };
	std::vector<float> right_pars	{ 60, 90, 90, 90 };

	dsv.left_pars = left_pars;
	dsv.leftp_pars = leftp_pars;
	dsv.front_pars = front_pars;
	dsv.rightp_pars = rightp_pars;
	dsv.right_pars = right_pars;
}

void init_output_speed_terms(speed_desition_space &dsv)
{
	std::vector<float> low_pars		{ 0, 0.5, 1, 1.5 };
	std::vector<float> middle_pars	{ 1, 2, 2.5, 3 };
	std::vector<float> high_pars	{ 2.5, 4, 5, 7 };
	dsv.low_pars = low_pars;
	dsv.middle_pars = middle_pars;
	dsv.high_pars = high_pars;
}

static void init_input(fl::InputVariable* in_var, float range_min, float range_max, std::string name)
{
	in_var->setName(name);
	in_var->setDescription("");
	in_var->setEnabled(true);
	in_var->setRange(range_min, range_max);
	in_var->setLockValueInRange(false);
}

static void init_output(fl::OutputVariable* out_var, float range_min, float range_max, std::string name)
{
	out_var->setName(name);
	out_var->setDescription("");
	out_var->setEnabled(true);
	out_var->setRange(range_min, range_max);
	out_var->setLockValueInRange(false);
	out_var->setAggregation(new fl::Maximum);
	out_var->setDefuzzifier(new fl::Centroid(100));
	out_var->setDefaultValue(fl::nan);
	out_var->setLockPreviousValue(false);
}

static void init_rule(fl::RuleBlock* rule_bl, std::string name)
{
	rule_bl->setName(name);
	rule_bl->setDescription("");
	rule_bl->setEnabled(true);
	rule_bl->setConjunction(fl::null);
	rule_bl->setDisjunction(fl::null);
	rule_bl->setImplication(new fl::AlgebraicProduct);
	rule_bl->setActivation(new fl::General);
}



void init_fuzzy(fl::Engine* engine, 
				fl::InputVariable* speed, 
				fl::InputVariable* obs_angle, 
				fl::OutputVariable* mSteer, 
				fl::OutputVariable* outSpeed,
				fl::RuleBlock* mamdani)
{
	obs_desition_space iodsv;	// input obstacle desition vector
	speed_desition_space isdsv;	// input speed desition vector
	obs_desition_space oodsv;	// output obstacle desition vector
	speed_desition_space osdsv;	// output speed desition vector

	//engine		= new fl::Engine;
	//speed		= new fl::InputVariable;
	//obs_angle	= new fl::InputVariable;
	//mSteer		= new fl::OutputVariable;
	//outSpeed	= new fl::OutputVariable;
	//mamdani		= new fl::RuleBlock;

	engine->setName("ObstacleAvoidance");
	engine->setDescription("");

	init_input(speed, 0, MAX_SPEED, "speed");
	init_input_speed_terms(isdsv);
	speed->addTerm(new fl::Trapezoid("slow",	isdsv.low_pars[0], isdsv.low_pars[1], isdsv.low_pars[2], isdsv.low_pars[3]));
	speed->addTerm(new fl::Trapezoid("middle",	isdsv.middle_pars[0], isdsv.middle_pars[1], isdsv.middle_pars[2], isdsv.middle_pars[3]));
	speed->addTerm(new fl::Trapezoid("fast",	isdsv.high_pars[0], isdsv.high_pars[1], isdsv.high_pars[2], isdsv.high_pars[3]));
	engine->addInputVariable(speed);

	init_input(obs_angle, -90, 90, "obstacle");
	init_input_obs_terms(iodsv);
	obs_angle->addTerm(new fl::Trapezoid("left",		iodsv.left_pars[0], iodsv.left_pars[1], iodsv.left_pars[2], iodsv.left_pars[3]));
	obs_angle->addTerm(new fl::Trapezoid("close_left",	iodsv.leftp_pars[0], iodsv.leftp_pars[1], iodsv.leftp_pars[2], iodsv.leftp_pars[3]));
	obs_angle->addTerm(new fl::Trapezoid("in_front",	iodsv.front_pars[0], iodsv.front_pars[1], iodsv.front_pars[2], iodsv.front_pars[3]));
	obs_angle->addTerm(new fl::Trapezoid("close_right", iodsv.rightp_pars[0], iodsv.rightp_pars[1], iodsv.rightp_pars[2], iodsv.rightp_pars[3]));
	obs_angle->addTerm(new fl::Trapezoid("right",		iodsv.right_pars[0], iodsv.right_pars[1], iodsv.right_pars[2], iodsv.right_pars[3]));
	engine->addInputVariable(obs_angle);

	init_output(mSteer, -90, 90, "robot_direction");
	init_output_obs_terms(oodsv);
	mSteer->addTerm(new fl::Trapezoid("left",		oodsv.left_pars[0],	 oodsv.left_pars[1], oodsv.left_pars[2], oodsv.left_pars[3]));
	mSteer->addTerm(new fl::Trapezoid("close_left", oodsv.leftp_pars[0], oodsv.leftp_pars[1], oodsv.leftp_pars[2], oodsv.leftp_pars[3]));
	mSteer->addTerm(new fl::Trapezoid("in_front",	oodsv.front_pars[0], oodsv.front_pars[1], oodsv.front_pars[2], oodsv.front_pars[3]));
	mSteer->addTerm(new fl::Trapezoid("close_right",oodsv.rightp_pars[0], oodsv.rightp_pars[1], oodsv.rightp_pars[2], oodsv.rightp_pars[3]));
	mSteer->addTerm(new fl::Trapezoid("right",		oodsv.right_pars[0], oodsv.right_pars[1], oodsv.right_pars[2], oodsv.right_pars[3]));
	engine->addOutputVariable(mSteer);

	init_output(outSpeed, 0, MAX_SPEED, "out_speed");
	init_output_speed_terms(osdsv);
	outSpeed->addTerm(new fl::Trapezoid("slow",	osdsv.low_pars[0], osdsv.low_pars[1], osdsv.low_pars[2], osdsv.low_pars[3]));
	outSpeed->addTerm(new fl::Trapezoid("middle", osdsv.middle_pars[0], osdsv.middle_pars[1], osdsv.middle_pars[2], osdsv.middle_pars[3]));
	outSpeed->addTerm(new fl::Trapezoid("fast",	osdsv.high_pars[0], osdsv.high_pars[1], osdsv.high_pars[2], osdsv.high_pars[3]));
	engine->addOutputVariable(outSpeed);

	init_rule(mamdani, "mamdani");
	mamdani->addRule(fl::Rule::parse("if obstacle is left then robot_direction is in_front", engine));
	mamdani->addRule(fl::Rule::parse("if obstacle is close_left then robot_direction is close_right", engine));
	mamdani->addRule(fl::Rule::parse("if obstacle is in_front then robot_direction is right", engine));
	mamdani->addRule(fl::Rule::parse("if obstacle is close_right then robot_direction is close_left", engine));
	mamdani->addRule(fl::Rule::parse("if obstacle is right then robot_direction is in_front", engine));

	mamdani->addRule(fl::Rule::parse("if obstacle is close_left then out_speed is middle", engine));
	mamdani->addRule(fl::Rule::parse("if obstacle is close_right then out_speed is middle", engine));
	mamdani->addRule(fl::Rule::parse("if obstacle is in_front then out_speed is slow", engine));
	mamdani->addRule(fl::Rule::parse("if obstacle is right then out_speed is fast", engine));
	mamdani->addRule(fl::Rule::parse("if obstacle is left then out_speed is fast", engine));

	engine->addRuleBlock(mamdani);

	std::string status;
	if (not engine->isReady(&status))
		throw fl::Exception("[engine error] engine is not ready:n" + status, FL_AT);
}

void init_fuzzy_ext(fl::Engine* engine,
					fl::InputVariable* speed,
					fl::InputVariable* obs_angle,
					fl::OutputVariable* mSteer,
					fl::OutputVariable* outSpeed,
					fl::RuleBlock* mamdani,
					variables_set	&var_s)
{
	obs_desition_space &iodsv = var_s.iodsv;	// input obstacle desition vector
	speed_desition_space &isdsv = var_s.isdsv;	// input speed desition vector
	obs_desition_space &oodsv = var_s.oodsv;	// output obstacle desition vector
	speed_desition_space &osdsv = var_s.osdsv;	// output speed desition vector

	engine->setName("ObstacleAvoidance");
	engine->setDescription("");

	init_input(speed, 0, MAX_SPEED, "speed");
	speed->addTerm(new fl::Trapezoid("slow", isdsv.low_pars[0], isdsv.low_pars[1], isdsv.low_pars[2], isdsv.low_pars[3]));
	speed->addTerm(new fl::Trapezoid("middle", isdsv.middle_pars[0], isdsv.middle_pars[1], isdsv.middle_pars[2], isdsv.middle_pars[3]));
	speed->addTerm(new fl::Trapezoid("fast", isdsv.high_pars[0], isdsv.high_pars[1], isdsv.high_pars[2], isdsv.high_pars[3]));
	engine->addInputVariable(speed);

	init_input(obs_angle, -90, 90, "obstacle");
	obs_angle->addTerm(new fl::Trapezoid("left", iodsv.left_pars[0], iodsv.left_pars[1], iodsv.left_pars[2], iodsv.left_pars[3]));
	obs_angle->addTerm(new fl::Trapezoid("close_left", iodsv.leftp_pars[0], iodsv.leftp_pars[1], iodsv.leftp_pars[2], iodsv.leftp_pars[3]));
	obs_angle->addTerm(new fl::Trapezoid("in_front", iodsv.front_pars[0], iodsv.front_pars[1], iodsv.front_pars[2], iodsv.front_pars[3]));
	obs_angle->addTerm(new fl::Trapezoid("close_right", iodsv.rightp_pars[0], iodsv.rightp_pars[1], iodsv.rightp_pars[2], iodsv.rightp_pars[3]));
	obs_angle->addTerm(new fl::Trapezoid("right", iodsv.right_pars[0], iodsv.right_pars[1], iodsv.right_pars[2], iodsv.right_pars[3]));
	engine->addInputVariable(obs_angle);

	init_output(mSteer, -90, 90, "robot_direction");
	mSteer->addTerm(new fl::Trapezoid("left", oodsv.left_pars[0], oodsv.left_pars[1], oodsv.left_pars[2], oodsv.left_pars[3]));
	mSteer->addTerm(new fl::Trapezoid("close_left", oodsv.leftp_pars[0], oodsv.leftp_pars[1], oodsv.leftp_pars[2], oodsv.leftp_pars[3]));
	mSteer->addTerm(new fl::Trapezoid("in_front", oodsv.front_pars[0], oodsv.front_pars[1], oodsv.front_pars[2], oodsv.front_pars[3]));
	mSteer->addTerm(new fl::Trapezoid("close_right", oodsv.rightp_pars[0], oodsv.rightp_pars[1], oodsv.rightp_pars[2], oodsv.rightp_pars[3]));
	mSteer->addTerm(new fl::Trapezoid("right", oodsv.right_pars[0], oodsv.right_pars[1], oodsv.right_pars[2], oodsv.right_pars[3]));
	engine->addOutputVariable(mSteer);

	init_output(outSpeed, 0, MAX_SPEED, "out_speed");
	outSpeed->addTerm(new fl::Trapezoid("slow", osdsv.low_pars[0], osdsv.low_pars[1], osdsv.low_pars[2], osdsv.low_pars[3]));
	outSpeed->addTerm(new fl::Trapezoid("middle", osdsv.middle_pars[0], osdsv.middle_pars[1], osdsv.middle_pars[2], osdsv.middle_pars[3]));
	outSpeed->addTerm(new fl::Trapezoid("fast", osdsv.high_pars[0], osdsv.high_pars[1], osdsv.high_pars[2], osdsv.high_pars[3]));
	engine->addOutputVariable(outSpeed);

	init_rule(mamdani, "mamdani");
	mamdani->addRule(fl::Rule::parse("if obstacle is left then robot_direction is in_front", engine));
	mamdani->addRule(fl::Rule::parse("if obstacle is close_left then robot_direction is close_right", engine));
	mamdani->addRule(fl::Rule::parse("if obstacle is in_front then robot_direction is right", engine));
	mamdani->addRule(fl::Rule::parse("if obstacle is close_right then robot_direction is close_left", engine));
	mamdani->addRule(fl::Rule::parse("if obstacle is right then robot_direction is in_front", engine));

	mamdani->addRule(fl::Rule::parse("if obstacle is close_left then out_speed is middle", engine));
	mamdani->addRule(fl::Rule::parse("if obstacle is close_right then out_speed is middle", engine));
	mamdani->addRule(fl::Rule::parse("if obstacle is in_front then out_speed is slow", engine));
	mamdani->addRule(fl::Rule::parse("if obstacle is right then out_speed is fast", engine));
	mamdani->addRule(fl::Rule::parse("if obstacle is left then out_speed is fast", engine));

	engine->addRuleBlock(mamdani);

	std::string status;
	if (not engine->isReady(&status))
		throw fl::Exception("[engine error] engine is not ready:n" + status, FL_AT);
}