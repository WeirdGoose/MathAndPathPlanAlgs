#pragma once

#include "gen_pars.h"

#define SPEED_TERMS_NUM 3
#define ANGLE_TERMS_NUM 3
#define TRIA_TERMS_SPACE_SIZE 50
#define SPEED_ACCESSORY_MAX 1 

typedef float _speed_space;


_speed_type speed_set_by_obs(	_speed_type max_speed,
								sens_det_dist_ dist_to_obs,
								sens_det_dist_ max_dist,
								_angle_type angle);
_speed_type speed_drop_by_obs(_speed_type max_speed,
								sens_det_dist_ dist_to_obs,
								sens_det_dist_ max_dist,
								_angle_type angle);
_speed_type fuzzy_speed_set(_speed_type max_speed,
							sens_det_dist_ actDistToObs,
							sens_det_dist_ maxDistToObs,
							std::vector<triangle_term_> terms);
_angle_type fuzzy_angle_set(sensor_point *sensor_points_ptr,
							obstacle_point &orient_repulsive,
							_sensor_num_type rob_lines_num,
							obstacle_point position);