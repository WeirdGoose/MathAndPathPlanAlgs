#pragma once

#include <iostream>
#include <list>    
#include <SFML/Graphics.hpp>
#include "gen_pars.h"
#include "ctrl_and_log.h"
// file access to that everyone has, here's also general defs and usings


#define GRID_VAL 1
#define ROB_ERROR_SPEED 0.2
#define ROB_ERROR_ROTATION 0.000112

#define MAX_MAP_SIZE_X 500
#define MAX_MAP_SIZE_Y 500
#define SCALE 1

class Whole_map
{
	std::vector<Column> map_points;
	//std::vector<Column> sensor_points;
	int8_t map_restriction = OBSTACLE_MAP_CHAR;
	int8_t& map_res = map_restriction;
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
	std::vector<map_point> short_obs;
	std::vector<obstacle_point> robs_positions;
	std::vector<_angle_type> orientation_angle;
	std::vector<line_obs> lines_on_map;
	std::vector<circle_obs> circles_on_map;
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
		if (i >= MAX_MAP_SIZE_X*SCALE 
			|| j >= MAX_MAP_SIZE_Y*SCALE)
			return map_res;
		else
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
	void update_short_obs(map_point anoth_point)
	{
		//write_log("obs at " + std::to_string(anoth_point.x) + " " + std::to_string(anoth_point.y));
		this->short_obs.push_back(anoth_point);
	}
	void add_point_with_width(map_point another_point)
	{
		const uint32_t wdth = 3;
		const uint32_t hght = 3;
		for (int i = 0; i < wdth; i++)
		{
			at(another_point.x - i, another_point.y) = OBSTACLE_MAP_CHAR;
			update_short_obs(map_point(another_point.x - i, another_point.y));
		}
		for (int i = 0; i < hght; i++)
		{
			at(another_point.x, another_point.y - i) = OBSTACLE_MAP_CHAR;
			update_short_obs(map_point(another_point.x, another_point.y - i));
		}
	}
	void create_quadro_obstacle(unsigned int width,
								unsigned int height,
								obstacle_point center_point, 
								int scale = 1)
	{
		map_point another_point;
		// init horizontal lines
		
		obstacles_weight += width * height;
		for (int j = 0; j < scale; ++j)
		{
			another_point.x = center_point.x - (width / 2) + width + 1;
			another_point.y = center_point.y + height / 2 + j;
			add_point_with_width(another_point);

			another_point.x = center_point.x + width / 2 + j;
			another_point.y = center_point.y - (height / 2) + height + 1;
			add_point_with_width(another_point);
		}

		for (unsigned int i = 0; i < width; i++)
		{
			for (int j = 0; j < scale; ++j)
			{
				another_point.x = center_point.x - width / 2 + i;
				another_point.y = center_point.y - height / 2 + j;
				add_point_with_width(another_point);
			}
			
			for (int j = 0; j < scale; ++j)
			{
				another_point.x = center_point.x - width / 2 + i;
				another_point.y = center_point.y + height / 2 + j;
				add_point_with_width(another_point);
			}

		}

		
		// init verical lines

		for (unsigned int i = 0; i < height; ++i)
		{
			for (int j = 0; j < scale; ++j)
			{
				another_point.x = center_point.x - width / 2 + j;
				another_point.y = center_point.y - height / 2 + i;
				add_point_with_width(another_point);
			}
			
			for (int j = 0; j < scale; ++j)
			{
				another_point.x = center_point.x + width / 2 + j;
				another_point.y = center_point.y - height / 2 + i;
				add_point_with_width(another_point);
			}
		}

		line_obs lineHorUp(	center_point.x - width / 2, center_point.y + height / 2, 
							center_point.x + width / 2, center_point.y + height / 2);

		line_obs lineHorDown(	center_point.x - width / 2, center_point.y - height / 2,
								center_point.x + width / 2, center_point.y - height / 2);

		line_obs lineVertLeft(	center_point.x - width / 2, center_point.y - height / 2,
								center_point.x - width / 2, center_point.y + height / 2);

		line_obs lineVertRight(	center_point.x + width / 2, center_point.y - height / 2,
								center_point.x + width / 2, center_point.y + height / 2);
		this->lines_on_map.push_back(lineHorUp);
		this->lines_on_map.push_back(lineHorDown);
		this->lines_on_map.push_back(lineVertLeft);
		this->lines_on_map.push_back(lineVertRight);
	}
	void create_line_obstacle(obstacle_point point1, obstacle_point point2)
	{
		map_point another_point;
		for (int i = 0; i < get_distance(point1, point2); i++)
		{
			float lambda;
			lambda = (i)/(get_distance(point1, point2) - i);
			another_point.x = (point1.x + lambda * point2.x) / (1 + lambda);
			another_point.y = (point1.y + lambda * point2.y) / (1 + lambda);
			add_point_with_width(another_point);
		}
		line_obs line(point1.x, point1.y, point2.x, point2.y);
		this->lines_on_map.push_back(line);
	}
	void create_circle_obstacle(obstacle_point center, float radius)
	{
		map_point another_point;
		for (float phi = 0; phi < 2 * M_PI; phi += 1 / (2 * M_PI*radius))
		{
			another_point.x = center.x + radius * cos(phi);
			another_point.y = center.y + radius * sin(phi);
			add_point_with_width(another_point);
		}
		circle_obs circle;
		circle.center.x = center.x;
		circle.center.y = center.y;
		circle.radius = radius;
		this->circles_on_map.push_back(circle);
	}
};

