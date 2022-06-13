#pragma once

#include <iostream>
#include <list>    
#include <SFML/Graphics.hpp>
#include "gen_pars.h"
// file access to that everyone has, here's also general defs and usings


#define GRID_VAL 1
#define ROB_ERROR_SPEED 0.2
#define ROB_ERROR_ROTATION 0.000112


class Whole_map
{
	std::vector<Column> map_points;
	//std::vector<Column> sensor_points;
	
	unsigned long size;
	unsigned int width;
	unsigned int height;
	unsigned int grid_scale = GRID_VAL;
	obstacle_point sight_dir;
private:
	void init(unsigned int width, unsigned int height) {
		
		this->size = width * height;
		this->width = width;
		this->height = height;
		for (unsigned int i = 0; i < this->width; ++i)
		{
			for (unsigned int j = 0; j < this->height; ++j)
			{
				this->map_points[i][j] = EMPTY_SPACE_MAP_CHAR;
			}
		}
	}
public:
	obstacle_point aim;
	obstacle_point rob_position;
	_angle_type orientation_angle;
	unsigned long obstacles_weight = 0;
	Whole_map(unsigned int width, unsigned int height) :
		map_points(width, Column(height)) {
			init(width, height);
		}
	Whole_map(unsigned int width, unsigned int height, unsigned int grid_scale) :
		map_points(width*grid_scale, Column(height*grid_scale)) {
			this->grid_scale = grid_scale;
			init(width*grid_scale, height*grid_scale);
		}
	
	int8_t& at(unsigned int i, unsigned int j)
	{
		return map_points[i][j];
	}
	unsigned long get(string parametr) 
	{
		if (!parametr.compare("width")) 
		{
			return this->width;
		}
		else if (!parametr.compare("height")) 
		{
			return this->height;
		}
		else if (!parametr.compare("size")) 
		{
			unsigned long sz = this->size;
			return sz;
		}
		else {
			cout << "Whole_map::get(): WARNING!!! UNKNOWN REQUEST PARAMETR \n";
			return 0;
		}
	}
	void create_quadro_obstacle(unsigned int width,
								unsigned int height,
								obstacle_point center_point, 
								int scale = 1)
	{
		map_point another_point;
		// init horizontal lines
		cout << "---------------quadro obs coodrinates--------------- \n\n\n";
		cout << "width \n\n\n";
		obstacles_weight += width * height;
		for (int j = 0; j < scale; ++j)
		{
			another_point.x = center_point.x - (width / 2) + width + 1;
			another_point.y = center_point.y + height / 2 + j;
			this->map_points[another_point.x][another_point.y] = OBSTACLE_MAP_CHAR;

			another_point.x = center_point.x + width / 2 + j;
			another_point.y = center_point.y - (height / 2) + height + 1;
			this->map_points[another_point.x][another_point.y] = OBSTACLE_MAP_CHAR;
		}

		for (unsigned int i = 0; i < width; i++)
		{
			for (int j = 0; j < scale; ++j)
			{
				another_point.x = center_point.x - width / 2 + i;
				another_point.y = center_point.y - height / 2 + j;
				this->map_points[another_point.x][another_point.y] = OBSTACLE_MAP_CHAR;
			}
			
			cout << "x - " << another_point.x << "\t| y - " << another_point.y << "\n-----------------\n";
			for (int j = 0; j < scale; ++j)
			{
				another_point.x = center_point.x - width / 2 + i;
				another_point.y = center_point.y + height / 2 + j;
				this->map_points[another_point.x][another_point.y] = OBSTACLE_MAP_CHAR;
			}
			cout << "x - " << another_point.x << "\t| y - " << another_point.y << "\n-----------------\n";

		}
		cout << "\n\n\nHEIGHT \n\n\n";
		// init verical lines

		for (unsigned int i = 0; i < height; ++i)
		{
			for (int j = 0; j < scale; ++j)
			{
				another_point.x = center_point.x - width / 2 + j;
				another_point.y = center_point.y - height / 2 + i;
				this->map_points[another_point.x][another_point.y] = OBSTACLE_MAP_CHAR;
			}
			
			cout << "x - " << another_point.x << "\t| y - " << another_point.y << "\n-----------------\n";
			for (int j = 0; j < scale; ++j)
			{
				another_point.x = center_point.x + width / 2 + j;
				another_point.y = center_point.y - height / 2 + i;
				this->map_points[another_point.x][another_point.y] = OBSTACLE_MAP_CHAR;
			}
			cout << "x - " << another_point.x << "\t| y - " << another_point.y << "\n-----------------\n";
		}
	}
};

