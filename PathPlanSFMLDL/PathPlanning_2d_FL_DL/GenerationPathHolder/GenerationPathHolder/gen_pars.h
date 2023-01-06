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

typedef uint8_t rob_pop_type_;
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

typedef struct obstacle_point_ {
	float x;
	float y;
}obstacle_point;

typedef struct map_point_ {
	unsigned int x;
	unsigned int y;
}map_point;

typedef struct sensor_point_ {
	obstacle_point pos;
	map_state_t_ state;
	sens_det_dist_ distant;
	//  DON'T depends orientation (placed in local coordinate system) left sensor points are negative, right - positive, center - 0
	_angle_type angle_from_center;
}sensor_point;

template <class one_p, class another_p>
float get_distance(one_p point1, another_p point2);