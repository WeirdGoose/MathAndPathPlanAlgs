#include "manu_fuzzy_logic.h"


_speed_type speed_set_by_obs(	_speed_type max_speed, 
								sens_det_dist_ dist_to_obs, 
								sens_det_dist_ max_dist, 
								_angle_type angle)
{
	_speed_type out_speed_set = 0;
	out_speed_set = max_speed * ((dist_to_obs / max_dist)) * (cos(angle));
	return out_speed_set;
}

_speed_type speed_drop_by_obs(	_speed_type max_speed,
								sens_det_dist_ dist_to_obs,
								sens_det_dist_ max_dist,
								_angle_type angle)
{
	_speed_type out_speed_drop;
	out_speed_drop = max_speed * (1 - (dist_to_obs / max_dist)) * cos(angle);
	return out_speed_drop;
}

_speed_type fuzzy_speed_set(_speed_type max_speed,
							sens_det_dist_ actDistToObs,
							sens_det_dist_ maxDistToObs,
							std::vector<triangle_term_> terms)
{
	//phase converting
	sens_det_dist_ distToObsCom = actDistToObs;
	std::vector<float> speed_accessory;
	int8_t i;
	int8_t it_of_max;
	float max_acessor = 0;
	size_t term_size = terms.size();
	speed_accessory.resize(term_size);

	if (distToObsCom <= terms[0].peak)
	{
		return 0;
	}
	for (i = 0; i < term_size; ++i)
	{
		speed_accessory[i] = 0;
		if ((distToObsCom >= terms[i].left_boarder) && (distToObsCom <= terms[i].peak))
		{
			speed_accessory[i] += 1 + SPEED_ACCESSORY_MAX * (terms[i].left_boarder - distToObsCom)/(terms[i].left_boarder - terms[i].peak);
		}
		else if ((distToObsCom >= terms[i].peak) && (distToObsCom <= terms[i].right_boarder))
		{
			speed_accessory[i] += 1 + SPEED_ACCESSORY_MAX * (terms[i].right_boarder - distToObsCom) / (terms[i].right_boarder - terms[i].peak);
		}
		if (max_acessor < speed_accessory[i])
		{
			max_acessor = speed_accessory[i];
			it_of_max = i;
		}
	}

	return terms[it_of_max].speed_nav;
}


_angle_type fuzzy_angle_set(sensor_point *sensor_points_ptr,
							obstacle_point &orient_repulsive,
							_sensor_num_type rob_lines_num,
							obstacle_point position,
							int sens_react_rad)
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
			orient_repulsive.x = (anoth_dist.x - position.x);
			orient_repulsive.y = (anoth_dist.y - position.y);
		}
	}
	return out_angle;
}

