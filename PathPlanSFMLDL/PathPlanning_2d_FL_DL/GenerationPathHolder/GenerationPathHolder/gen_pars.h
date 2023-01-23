#pragma once

#include <iostream>
#include <list>    
#include <SFML/Graphics.hpp>
#include <thread>
#include <windows.h>
//#define _USE_MATH_DEFINES
#include <math.h>
#include <corecrt_math_defines.h>

using std::cout;
using std::cin;
using std::endl;
using std::string;
using std::list;

#define MAX(x, y) x > y ? x : y
#define MIN(x, y) x <= y ? x: y
#define __int64 long long
#define EMPTY_SPACE_MAP_CHAR 0
#define OBSTACLE_MAP_CHAR 1
#define ROBOT_MAP_CHAR 2
#define ROBOT_PATH_MAP_CHAR 3

enum active_cyc_mode {
	_speed_setting_,
	_rotation_setting_,
	_fuzzy_set_
};

typedef float sens_det_dist_;
typedef float _angle_type;
typedef float _speed_type;
typedef int8_t map_state_t_;
typedef uint8_t _sensor_num_type;

typedef struct {
	float peak;
	float right_boarder;
	float left_boarder;
	float speed_nav;	// the output speed value, thats corresponding to term 
} triangle_term_;

typedef std::vector<map_state_t_> Column;

typedef struct map_point_ {
	unsigned int x;
	unsigned int y;
}map_point;


class obstacle_point {
public:
	float x;
	float y;
	obstacle_point() {}
	obstacle_point(float x, float y)
	{
		this->x = x;
		this->y = y;
	}
};

typedef struct sensor_point_ {
	obstacle_point pos;
	map_state_t_ state;
	sens_det_dist_ distant;
	//  DON'T depends orientation (placed in local coordinate system) left sensor points are negative, right - positive, center - 0
	_angle_type angle_from_center;
}sensor_point;


class line_obs {
public:
	obstacle_point point1;
	obstacle_point point2;
	line_obs() {}
	line_obs(float p1x, float p1y, float p2x, float p2y)
	{
		this->point1.x = p1x;
		this->point1.y = p1y;
		this->point2.x = p2x;
		this->point2.y = p2y;
	}

};

typedef struct circle_obs_ {
	obstacle_point center;
	float radius;
}circle_obs;


template <class one_p, class another_p>
float get_distance(one_p point1, another_p point2);
