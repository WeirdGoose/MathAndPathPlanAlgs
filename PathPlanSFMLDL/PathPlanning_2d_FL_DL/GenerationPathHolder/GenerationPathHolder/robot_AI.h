#pragma once
#include "Whole_map.h"
#include "gen_pars.h"


// in whole_map coordinate system
#define START_ROBOT_POS_X 300
#define START_ROBOT_POS_Y 300
#define ROBOT_SIGHT_POINT_X START_ROBOT_POS_X + 1
#define ROBOT_SIGHT_POINT_Y START_ROBOT_POS_Y + 1
#define AIM_POS_X 190
#define AIM_POS_Y 190
#define START_ANGLE 0
#define START_DELTA_T 0.5		// seems to be float
#define MAX_SPEED	7.0
#define MAX_SPEED_2 MAX_SPEED*MAX_SPEED
#define START_SPEED MAX_SPEED

#define LINES_NUMBER 11
#define LINES_RADIUS 30
#define NUM_OF_SENS_CHK_STEPS 30
#define SENSOR_RAD M_PI
#define SENSOR_RAD_STEP SENSOR_RAD/(LINES_NUMBER)
#define ORIENT_DIST LINES_RADIUS
#define AIM_RADIUS	6




class robot_params 
{
	sensor_point sensor_points[LINES_NUMBER];
	obstacle_point orientation;
	_speed_type Speed;

	// only for odd LINES_NUMBER for now
	void direct_sensors(float angle, obstacle_point rob_pos)
	{
		//cout << "---------------sensor lines coodrinates redir--------------- \n\n\n";
		// right
		for (_sensor_num_type i = 0; i < LINES_NUMBER/2 + 1; ++i)
		{
			this->sensor_points[i].pos.x = rob_pos.x + LINES_RADIUS * cos(SENSOR_RAD_STEP * i + angle);
			this->sensor_points[i].pos.y = rob_pos.y + LINES_RADIUS * sin(SENSOR_RAD_STEP * i + angle);
			this->sensor_points[i].state = EMPTY_SPACE_MAP_CHAR;
			this->sensor_points[i].distant = 0;
			//cout << "x - " << this->sensor_points[i].pos.x << "\t| y - " << this->sensor_points[i].pos.y << "\n-----------------\n";
		}
		
		// left
		for (_sensor_num_type i = LINES_NUMBER / 2 + 1; i < LINES_NUMBER; ++i)
		{
			this->sensor_points[i].pos.x = rob_pos.x + LINES_RADIUS * cos(-SENSOR_RAD_STEP * (i - LINES_NUMBER / 2) + angle);
			this->sensor_points[i].pos.y = rob_pos.y + LINES_RADIUS * sin(-SENSOR_RAD_STEP * (i - LINES_NUMBER / 2) + angle);
			this->sensor_points[i].state = EMPTY_SPACE_MAP_CHAR;
			this->sensor_points[i].distant = 0;
			//cout << "x - " << this->sensor_points[i].pos.x << "\t| y - " << this->sensor_points[i].pos.y << "\n-----------------\n";
		}
	}
public:
	std::vector<sens_det_dist_> sens_math_lambdas;	// array of rate vectors for every n compare points  of 1 sensor line(or sensor point)
	std::vector<triangle_term_> terms;
	unsigned long path_mem_size = 1;
	float delta_t;
	obstacle_point aim;
	obstacle_point position;
	_angle_type orientation_angle;
	obstacle_point orient_attractive;
	obstacle_point orient_repulsive;
	unsigned int rob_lines_num = LINES_NUMBER;
	robot_params(){}
	obstacle_point& at(unsigned int i)
	{
		return this->sensor_points[i].pos;
	}
	void set_speed(_speed_type req_speed)
	{
		this->Speed = req_speed;
	}
	_speed_type get_speed()
	{
		return this->Speed;
	}
	obstacle_point get_orientation()
	{
		return this->orientation;
	}
	void sensors_trigg(_sensor_num_type sensor_number, int8_t distant, map_state_t_ state)
	{
		this->sensor_points[sensor_number].state = state;
		this->sensor_points[sensor_number].distant = distant;
	}
	void set_direction(obstacle_point direction)
	{
		this->orientation.x = direction.x;
		this->orientation.y = direction.y;
		
		this->orientation_angle = M_PI + atan2((this->position.y - this->orientation.y) , (this->position.x - this->orientation.x));
		
		cout << "angle " << this->orientation_angle << "\n";
		//direct_sensors(this->orientation_angle, this->position);
	}
	// changing sensors direction caused by rotation of robot
	void set_direction_by_angle(_angle_type angle)
	{
		this->orientation_angle = angle;
		this->orientation.x = this->position.x + ORIENT_DIST * sin(angle);
		this->orientation.y = this->position.y + ORIENT_DIST * cos(angle);
		direct_sensors(angle, this->position);
	}
	void set_start_pos_and_dir(obstacle_point pos, _angle_type angle)
	{
		this->position.x = pos.x;
		this->position.y = pos.y;
		cout << "robot_params:: rob position x, y is " << pos.x << ", " << pos.y << "\n";
		//set_direction_by_angle(angle);
		this->sens_math_lambdas.resize(NUM_OF_SENS_CHK_STEPS);
		// setting lambdas (there can be linear or another correlation)
		for (int i = sens_math_lambdas.size() - 1; i >= 0; --i)
		{
			this->sens_math_lambdas[i] = (float)i/((float)LINES_RADIUS - (float)i);
			cout << i << " sens_math_lambda " << sens_math_lambdas[i] << " \n";
		}
		for (int8_t i = 0; i < LINES_NUMBER / 2 + 1; ++i)
		{
			this->sensor_points[i].angle_from_center = i * SENSOR_RAD_STEP;
			cout << "ang - " << this->sensor_points[i].angle_from_center << "\n";
		}
		for (int8_t i = LINES_NUMBER / 2 + 1; i < LINES_NUMBER; ++i)
		{
			this->sensor_points[i].angle_from_center = ((LINES_NUMBER / 2) - i) * SENSOR_RAD_STEP;
			cout << "ang - " << this->sensor_points[i].angle_from_center << "\n";
		}
	}
	sensor_point * get_sensor_points()
	{
		return this->sensor_points;
	}
};

void init_logic(Whole_map &map, robot_params &rob_base);
void calc_orient_repulsive(	sensor_point *sensor_points_ptr,
							obstacle_point &orient_repulsive,
							_sensor_num_type rob_lines_num,
							obstacle_point position);
BOOL robot_active_cyc(Whole_map &map, robot_params &rob_base, enum active_cyc_mode mode);
