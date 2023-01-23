#include "robot_Ai.h"
#include "gen_pars.h"
#include "manu_fuzzy_logic.h"
#include "fl/Headers.h"


uint8_t flag_one = 0;
//	robot logic description
//	robot_params rob_base;
const float pos_sc_fact = 4;
const float rob_rad = 1;
const int sens_react_rad = 29;


void init_logic(Whole_map &map, robot_params &rob_base) 
{
	obstacle_point start_pos;
	start_pos.x = START_ROBOT_POS_X;
	start_pos.y = START_ROBOT_POS_Y;

	rob_base.delta_t = START_DELTA_T;

	rob_base.aim.x = AIM_POS_X;
	rob_base.aim.y = AIM_POS_Y;
	rob_base.set_speed(START_SPEED);
	rob_base.set_start_pos_and_dir(start_pos, START_ANGLE);
	rob_base.terms.resize(SPEED_TERMS_NUM);
	
	rob_base.engine =		new fl::Engine;
	//rob_base.fl_aim_angle =	new fl::InputVariable;
	rob_base.fl_obs_angle = new fl::InputVariable;
	rob_base.mSteer =		new fl::OutputVariable;
	rob_base.fl_outSpeed =	new fl::OutputVariable;
	rob_base.mamdani =		new fl::RuleBlock;
	rob_base.fl_distance =	new fl::InputVariable;

	init_fuzzy(rob_base.engine, 
		rob_base.fl_distance,
		rob_base.fl_obs_angle, 
		rob_base.mSteer, 
		rob_base.fl_outSpeed, 
		rob_base.mamdani);
	
	cout << "|i \t|left board \t|peak \t|right board \t|\n";
	for (unsigned i = 0; i < rob_base.terms.size(); ++i)
	{
		rob_base.terms[i].peak = LINES_RADIUS / 6 +i*LINES_RADIUS/ rob_base.terms.size();
		rob_base.terms[i].right_boarder = rob_base.terms[i].peak + LINES_RADIUS / 6;
		rob_base.terms[i].left_boarder = rob_base.terms[i].peak - LINES_RADIUS / 6;
		rob_base.terms[i].speed_nav = MAX_SPEED - MAX_SPEED/(i + 1) + MAX_SPEED/ rob_base.terms.size();
		cout << "|" << rob_base.terms[i].left_boarder << "|" << rob_base.terms[i].peak << "|" << rob_base.terms[i].right_boarder << "|\n";
	}
	rob_base.orient_repulsive.y = 0;
	rob_base.orient_repulsive.x = 0;
}

float sens_loc_angle_by_num(_sensor_num_type num)
{
	float angle;
	if (num < LINES_NUMBER / 2 + 1)
	{
		angle = SENSOR_RAD_STEP * num;
	}
	else if (LINES_NUMBER / 2 + 1 <= num < LINES_NUMBER)
	{
		angle  = -SENSOR_RAD_STEP * (num - LINES_NUMBER / 2);
	}
	else {
		cout << "sens_angle_by_num: wrong number!!!!\n";
		return 0;
	}
	return angle*180/M_PI;
}

float orient_from_local_angle(float angle, float currGlobalAngle)
{
	if (std::isnan(angle))
		cout << "";
	return currGlobalAngle + angle*M_PI/180;
}


void calc_orient_repulsive(	sensor_point *sensor_points_ptr, 
							obstacle_point &orient_repulsive, 
							_sensor_num_type rob_lines_num, 
							obstacle_point position)
{
	obstacle_point anoth_dist;	
	float norm_scal;
	orient_repulsive.x = 0;
	orient_repulsive.y = 0;

	for (_sensor_num_type i = 0; i < rob_lines_num; ++i)
	{
		if (sensor_points_ptr[i].state == OBSTACLE_MAP_CHAR
			&& sensor_points_ptr[i].distant < sens_react_rad)
		{
			norm_scal = (sens_react_rad - sensor_points_ptr[i].distant) / sensor_points_ptr[i].distant;
			anoth_dist.x = (position.x + norm_scal * sensor_points_ptr[i].pos.x) / (1 + norm_scal);
			anoth_dist.y = (position.y + norm_scal * sensor_points_ptr[i].pos.y) / (1 + norm_scal);
			orient_repulsive.x = pos_sc_fact * (anoth_dist.x - position.x);
			orient_repulsive.y = pos_sc_fact * (anoth_dist.y - position.y);
		}
	}
}

void calc_orient_attractive(robot_params &rob_base)
{
	obstacle_point orient_attractive;
	float normal;
	float lambda_attr;
	orient_attractive.y = rob_base.position.y - rob_base.aim.y;
	orient_attractive.x = rob_base.position.x - rob_base.aim.x;
	normal = sqrt(pow(orient_attractive.x, 2) + pow(orient_attractive.y, 2));
	lambda_attr = sens_react_rad / (normal - sens_react_rad);
	orient_attractive.y = (rob_base.position.y + lambda_attr * rob_base.aim.y) / (lambda_attr + 1);
	orient_attractive.x = (rob_base.position.x + lambda_attr * rob_base.aim.x) / (lambda_attr + 1);

	rob_base.orient_attractive.y = orient_attractive.y - rob_base.orient_repulsive.y;
	rob_base.orient_attractive.x = orient_attractive.x - rob_base.orient_repulsive.x;
}


// setting direction and speed (with phase control logic and etc)
// if movement is cyclic, then setting direction must be also cyclic
BOOL robot_active_cyc(Whole_map &map, robot_params &rob_base, enum active_cyc_mode mode)
{
	if (rob_base.position.x <= rob_base.aim.x + AIM_RADIUS
		&& rob_base.position.x >= rob_base.aim.x - AIM_RADIUS)
	{
		if (rob_base.position.y <= rob_base.aim.y + AIM_RADIUS
			&& rob_base.position.y >= rob_base.aim.y - AIM_RADIUS)
		{
			return 1;
		}
	}
	if (mode == _rotation_setting_)
	{
		sensor_point *sensor_points_ptr;
		//sens_det_dist_ norm_scale = rob_base.sens_math_lambdas[sens_react_rad - 1];
		sensor_points_ptr = rob_base.get_sensor_points();
		calc_orient_repulsive(sensor_points_ptr, rob_base.orient_repulsive, rob_base.rob_lines_num, rob_base.position);
		calc_orient_attractive(rob_base);
		rob_base.set_direction(rob_base.orient_attractive);
	}
	else if (mode == _fuzzy_set_)
	{
		sensor_point *sensor_points_ptr;
		uint8_t num = 0;
		float min_dist = LINES_RADIUS;
		sensor_points_ptr = rob_base.get_sensor_points();

		for (_sensor_num_type i = 0; i < rob_base.rob_lines_num; ++i)
		{
			if (sensor_points_ptr[i].state == OBSTACLE_MAP_CHAR
				&& sensor_points_ptr[i].distant < LINES_RADIUS - 1)
			{
				if (min_dist > sensor_points_ptr[i].distant)
				{
					min_dist = sensor_points_ptr[i].distant;
					num = i;
				}
			}
		}
		if (min_dist != LINES_RADIUS)
		{
			float angle_ctrl = sens_loc_angle_by_num(num);
			//rob_base.fl_aim_angle->setValue(rob_base.get_speed());

			rob_base.fl_obs_angle->setValue(angle_ctrl);
			rob_base.fl_distance->setValue(min_dist);
			rob_base.engine->process();

			cout << "out angle value is " << rob_base.mSteer->getValue() << "\n";

			float out_glob_angle = orient_from_local_angle(rob_base.mSteer->getValue(), rob_base.orientation_angle);
			
			cout << "in angle is " << angle_ctrl << " fl angle is "
				<< fl::Op::str(rob_base.fl_obs_angle->getValue()) << "\n";

			cout << "out angle is " << rob_base.orientation_angle << " fl angle is "
				<< fl::Op::str(rob_base.mSteer->getValue()) << "\n";
			rob_base.set_direction_by_angle(out_glob_angle);

			rob_base.set_speed(rob_base.fl_outSpeed->getValue());

		}
		else
		{
			calc_orient_attractive(rob_base);
			rob_base.set_direction(rob_base.orient_attractive);
			rob_base.set_speed(MAX_SPEED);
		}
		
	}
	
	return 0;
}