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


typedef float sens_det_dist_;
typedef float _angle_type;
typedef float _speed_type;
typedef int8_t map_state_t_;
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