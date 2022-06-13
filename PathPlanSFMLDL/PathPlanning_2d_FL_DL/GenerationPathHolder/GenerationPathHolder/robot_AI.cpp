#include "robot_Ai.h"
#include "gen_pars.h"
#include "manu_fuzzy_logic.h"
#include "genetic_algh.h"

uint8_t flag_one = 0;
//	robot logic description
//	robot_params rob_base;
const float pos_sc_fact = 1.0;
const float rob_rad = 1;
const int sens_react_rad = 30;


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


// checked is there are obstacle by compare points of every sensor line with obstacle coordinates
sensor_point *check_sensors(Whole_map &map, robot_params &rob_base)
{
	sensor_point *sensor_points_ptr;
	sensor_points_ptr = rob_base.get_sensor_points();
	sensor_point sens_obs;
	map_point sens_disc;
	map_point rob_pos_disc;
	
	for (_sensor_num_type i = 0; i < rob_base.rob_lines_num; ++i)
	{
		sens_obs = sensor_points_ptr[i];
		rob_base.sensors_trigg(i, LINES_RADIUS+1, EMPTY_SPACE_MAP_CHAR);
		for (int8_t j = 0; j < rob_base.sens_math_lambdas.size(); ++j)
		{
			sens_disc.x = (rob_base.position.x + rob_base.sens_math_lambdas[j] * sens_obs.pos.x)/(1 + rob_base.sens_math_lambdas[j]);
			sens_disc.y = (rob_base.position.y + rob_base.sens_math_lambdas[j] * sens_obs.pos.y)/(1 + rob_base.sens_math_lambdas[j]);
			if (map.at(sens_disc.x, sens_disc.y) == OBSTACLE_MAP_CHAR)
			{
				rob_base.sensors_trigg(i, j, OBSTACLE_MAP_CHAR);
				flag_one = 1;
				break;
			}
		}
	}
	// return the pointer to sensor_array
	return sensor_points_ptr;
}

void calc_orient_repulsive(	sensor_point *sensor_points_ptr, 
							obstacle_point &orient_repulsive, 
							_sensor_num_type rob_lines_num, 
							obstacle_point position)
{
	obstacle_point anoth_dist;	
	float norm_scal;

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


// setting direction and speed (with phase control logic and etc)
// if movement is cyclic, then setting direction must be also cyclic
void robot_active_cyc(Whole_map &map, robot_params &rob_base) 
{
	
	sensor_point *sensor_points_ptr;
	obstacle_point orient_attractive;
	//sens_det_dist_ norm_scale = rob_base.sens_math_lambdas[sens_react_rad - 1];
	sensor_points_ptr = rob_base.get_sensor_points();
	float normal;
	float lambda_attr;
	calc_orient_repulsive(	sensor_points_ptr, rob_base.orient_repulsive, rob_base.rob_lines_num, rob_base.position);
	
	orient_attractive.y = rob_base.position.y - rob_base.aim.y;
	orient_attractive.x = rob_base.position.x - rob_base.aim.x;
	normal = sqrt(pow(orient_attractive.x, 2) + pow(orient_attractive.y, 2));
	lambda_attr = sens_react_rad / (normal - sens_react_rad);
	orient_attractive.y = (rob_base.position.y + lambda_attr*rob_base.aim.y) / (lambda_attr + 1);
	orient_attractive.x = (rob_base.position.x + lambda_attr*rob_base.aim.x) / (lambda_attr + 1);

	rob_base.orient_attractive.y = orient_attractive.y - rob_base.orient_repulsive.y;
	rob_base.orient_attractive.x = orient_attractive.x - rob_base.orient_repulsive.x;
	cout << "orient_attractive.x " << orient_attractive.x << " orient_attractive.y " << orient_attractive.y << "\n";
	cout << "orient_repulsive.x " << rob_base.orient_repulsive.x << " orient_repulsive.y " << rob_base.orient_repulsive.y << "\n";

	rob_base.set_direction(rob_base.orient_attractive);
	_speed_type speed = rob_base.get_speed();
	
	sensor_points_ptr = check_sensors(map, rob_base);

	
	if (sensor_points_ptr[0].state == OBSTACLE_MAP_CHAR)
	{
		speed = fuzzy_speed_set(MAX_SPEED, sensor_points_ptr[0].distant, LINES_RADIUS, rob_base.terms);
		cout << "in slowing: speed - " << speed << "\n";
		cout << "in slowing: dist to obs - " << sensor_points_ptr[0].distant << "\n";
		if (speed <= 0)
		{
			cout << "speed <= 0!!!\n";
			speed = 0;
		}
	}
	rob_base.set_speed(speed);

}