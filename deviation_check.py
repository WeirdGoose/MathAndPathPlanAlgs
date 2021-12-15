import math
import tp_dest as tpd

# Константы
First_Point = 0
Null_vector = [0.0, 0.0, 0.0]
Corridor_Radius = 8
# Переменные
copter_coord_after_takeoff = Null_vector

# --------------------------------------------------------------------------------------------------
# Функции Внутренние

# Перевод из глобальных координат в локальные. Начало координат - квадрокоптер
# Возвращает список [координаты коптера(нулевой вектор), предыдущую точку-цель, следующую точку-цель]
def proceed_coordinate_system(copter_coord_gps, point_previous, point_next):
    previous_point = tpd.enu_vector(copter_coord_gps, point_previous)
    current_point = tpd.enu_vector(copter_coord_gps, point_next)
    return [Null_vector, previous_point, current_point]

# Расчет расстояния по https://ru.onlinemschool.com/math/library/analytic_geometry/p_line/
# Возвращает расстояние до траектории
def distance_calculation(copter_coord, point_previous, point_next):
    # Инициализация
    directing_vector = [0.0, 0.0, 0.0]
    Vector_Copter_Goal = [0.0, 0.0, 0.0]
    Vector_C_G_S = [0.0, 0.0, 0.0]
    # Расчет расстояния 
    for i in range(len(copter_coord)):
        directing_vector[i] = point_next[i] - point_previous[i]
        Vector_Copter_Goal[i] = point_next[i] - copter_coord[i]

    Vector_C_G_S[0] = Vector_Copter_Goal[1] * directing_vector[2] - Vector_Copter_Goal[2] * directing_vector[1]
    Vector_C_G_S[1] = - ( Vector_Copter_Goal [0] * directing_vector[2] - Vector_Copter_Goal[2] * directing_vector[0] )
    Vector_C_G_S[2] = Vector_Copter_Goal[0] * directing_vector[1] - Vector_Copter_Goal[1] * directing_vector[0]
    
    directing_vector_length = math.hypot(directing_vector[0], directing_vector[1], directing_vector[2])

    Vector_C_G_S_length = math.hypot(Vector_C_G_S[0], Vector_C_G_S[1], Vector_C_G_S[2])
    destination = Vector_C_G_S_length/directing_vector_length

    return destination


# Возвращает координаты точки на траектории полёта http://www.cleverstudents.ru/line_and_plane/projection_of_point_onto_line.html
def normal_point_calculation_xyz(copter_coord, point_previous, point_next):
    # Инициализация
    d_plane_coef = 0.0
    free_coefficient = 0.0
    directing_vector = [0.0, 0.0, 0.0]
    searched_point = [0.0, 0.0, 0.0]
    poin_on_line = [0.0, 0.0, 0.0]
    # Расчет направляющего вектора прямой
    for i in range(len(copter_coord)):
        directing_vector[i] = point_next[i] - point_previous[i]
        poin_on_line[i] = point_previous[i]
    # Расчет коэффицентов плоскости, через направляющий вектор
    a_plane_coef = directing_vector[0]
    b_plane_coef = directing_vector[1]
    c_plane_coef = directing_vector[2]
    
    d_plane_coef = -(a_plane_coef * copter_coord[0]
    + b_plane_coef * copter_coord[1]
    + c_plane_coef * copter_coord[2])
    # Находим проекцию
    free_coefficient = (d_plane_coef 
                    + a_plane_coef * poin_on_line[0]
                    + b_plane_coef * poin_on_line[1] 
                    + c_plane_coef * poin_on_line[2])
        
    lambda_coef = - free_coefficient/(a_plane_coef * directing_vector[0] 
                                      + b_plane_coef * directing_vector[1] 
                                      + c_plane_coef * directing_vector[2])

    for i in range(len(copter_coord)):
        searched_point[i] = lambda_coef*directing_vector[i] + poin_on_line[i]
    return searched_point

def deviation_calculation(copter_coord, point_to_move):
    deviation = [0, 0, 0]
    deviation = [point_to_move[0] - copter_coord[0], 
                 point_to_move[1] - copter_coord[1], 
                 point_to_move[2] - copter_coord[2]]
    return deviation

# Расчет граничной точки отклонения от траектории
# Возвращает локальные координаты точки-коридора, или вектор до точки-коридора от коптера (это одно и тоже) 
def corridor_point_calculation_xyz(copter_coord, point_previous, point_next):
    
    deviation = [0.0, 0.0, 0.0]
    corridor_point = [0.0, 0.0, 0.0]
    point_to_move = normal_point_calculation_xyz(copter_coord, point_previous, point_next)
    print("point_to_move is ", point_to_move)
    deviation = deviation_calculation(copter_coord, point_to_move)    
    print("deviation is ", deviation)
    for i in range(len(copter_coord)):
        deviation[i] = - deviation[i]
    
    for_normal = math.hypot(deviation[0], deviation[1], deviation[2])
    print("for_normal is ", for_normal)
    if for_normal == 0: # Если коптер на траектории
        corridor_point = point_previous
        return corridor_point
    else:
        for i in range(len(copter_coord)):
            deviation[i] = deviation[i]/for_normal
            corridor_point[i] = (Corridor_Radius - for_normal)*deviation[i]
        print("Corridor_Radius - for_normal is ", Corridor_Radius - for_normal)
        return corridor_point
        

# ------------------------------------------------------------------------------------------------------------
# Функции внешние 
# Эти функции работают только если список точек-целей заполняется в ходе цикла, а не сначала

# Функция возвращает значение отклонения в виде 1 значения расстояния в метрах
def get_distance(goal_global_gps = [], goal_number = 0, copter_coord_gps = [0, 0, 0]):
    # goal_global_xyz - весь существующий список точек-целей
    
    # TODO сделать защиту от занесения в goal_global_xyz НЕ списка-списков, а обычного
    # Инициализация
    global copter_coord_after_takeoff
    copter_xyz = []
    distance = 0.0
    goal_xyz_prev = [0, 0, 0]
    goal_xyz_next = [0, 0, 0]
    # Основной вызов алгоритмов
    if len(goal_global_gps) == 0: # Особенность внешнего алгоритма (выполнение условия означает, что коптер только взлетает)
        # во время взлёта считывается gps коптера и перед полётом к цели делается предыдущей точкой
        copter_coord_after_takeoff = copter_coord_gps
    elif goal_number == First_Point:
        copter_xyz, goal_xyz_prev, goal_xyz_next = proceed_coordinate_system(copter_coord_gps, 
                                                                             copter_coord_after_takeoff, 
                                                                             goal_global_gps[goal_number])
        dist_tmp = math.hypot(abs(copter_xyz[0]-goal_xyz_next[0]), 
                                abs(copter_xyz[1]-goal_xyz_next[1]), 
                                abs(copter_xyz[2]-goal_xyz_next[2]))
        if dist_tmp < 8:
            distance = dist_tmp
        else:
            distance = distance_calculation(copter_xyz, goal_xyz_prev, goal_xyz_next)
    else:
        copter_xyz, goal_xyz_prev, goal_xyz_next = proceed_coordinate_system(copter_coord_gps,
                                                                             goal_global_gps[goal_number - 1], 
                                                                             goal_global_gps[goal_number])
        dist_tmp = math.hypot(abs(copter_xyz[0]-goal_xyz_next[0]), 
                                abs(copter_xyz[1]-goal_xyz_next[1]), 
                                abs(copter_xyz[2]-goal_xyz_next[2]))
        if dist_tmp < 8:
            distance = dist_tmp
        else:
            distance = distance_calculation(copter_xyz, goal_xyz_prev, goal_xyz_next)
    return distance


# Функция возвращает значение отклонения в виде расстояния в метрах по x, y, z
def get_deviation(goal_global_gps = [], goal_number = 0, copter_coord_gps = [0, 0, 0]):
    # goal_global_xyz - весь существующий список точек-целей
    
    # TODO сделать защиту от занесения в goal_global_xyz НЕ списка-списков, а обычного
    # Инициализация
    global copter_coord_after_takeoff
    copter_xyz = []
    deviation = [0, 0, 0]
    goal_xyz_prev = [0, 0, 0]
    goal_xyz_next = [0, 0, 0]
    point_to_move = [0, 0, 0]
    # Основной вызов алгоритмов
    if len(goal_global_gps) == 0:
        copter_coord_after_takeoff = copter_coord_gps
        print("copter still didn't takeoffs (it's probably okay btw)")
    elif goal_number == First_Point:
        copter_xyz, goal_xyz_prev, goal_xyz_next = proceed_coordinate_system(copter_coord_gps, 
                                                                             copter_coord_after_takeoff, 
                                                                             goal_global_gps[goal_number])
        point_to_move = normal_point_calculation_xyz(copter_xyz, goal_xyz_prev, goal_xyz_next)
        deviation = deviation_calculation(copter_xyz, point_to_move)
    else:
        copter_xyz, goal_xyz_prev, goal_xyz_next = proceed_coordinate_system(copter_coord_gps, 
                                                                            goal_global_gps[goal_number - 1], 
                                                                            goal_global_gps[goal_number])
        point_to_move = normal_point_calculation_xyz(copter_xyz, goal_xyz_prev, goal_xyz_next)
        deviation = deviation_calculation(copter_xyz, point_to_move)
    #print("points is ", copter_xyz, "\t", goal_xyz_prev, "\t", goal_xyz_next)
    return deviation

# Функция коридора
# Возвращает координату точки в поверхности коридора, за который нельзя вылетать, и делает это в локальных координатах, 
# т.е. по факту это расстояние по x, y, z отк коптера до точки коридора
def get_corridor_point(goal_global_gps = [], goal_number = 0, copter_coord_gps = [0, 0, 0]):
    global copter_coord_after_takeoff
    copter_xyz = []
    corridor_point = [0, 0, 0]
    goal_xyz_prev = [0, 0, 0]
    goal_xyz_next = [0, 0, 0]
    # Основной вызов алгоритмов
    if len(goal_global_gps) == 0:
        copter_coord_after_takeoff = copter_coord_gps
    elif goal_number == First_Point:
        copter_xyz, goal_xyz_prev, goal_xyz_next = proceed_coordinate_system(copter_coord_gps, 
                                                                             copter_coord_after_takeoff, 
                                                                             goal_global_gps[goal_number])
        corridor_point = corridor_point_calculation_xyz(copter_xyz, goal_xyz_prev, goal_xyz_next)
    else:
        copter_xyz, goal_xyz_prev, goal_xyz_next = proceed_coordinate_system(copter_coord_gps, 
                                                                            goal_global_gps[goal_number - 1], 
                                                                            goal_global_gps[goal_number])
        corridor_point = corridor_point_calculation_xyz(copter_xyz, goal_xyz_prev, goal_xyz_next)
    print("points is ", copter_xyz, "\t", goal_xyz_prev, "\t", goal_xyz_next)
    return corridor_point
    

# TODO Оптимизация
# 1. заменить все for ... на map
# 2. подумать, чо сделать с if else
# 3. попроовать записать математику в одну строку
# -------------------------------------------------------------------------------------------------
# Тест функция
# TODO после тестов убрать
if __name__ == '__main__':
    goal_global_ = [[1.0046790641871626, 1.0009882945059274, 120.0]]
    copt_coord = [1.0044473, 1.0009254, 111.2544131963978]
    copt_coord2 = [1.0044473, 1.0009254, 125.2544131963978]
    copt_coord3 = [1.0046890641871626, 1.0009982945059274, 120.0]
    
    """
    #-----------------тест distance----------------------------
    d = get_distance([], 0, copt_coord)
    d = get_distance(goal_global_, 0, copt_coord)
    print("get_distance is ", d)
    print("copter_coord_after_takeoff is ", copter_coord_after_takeoff)
    print()
    
    #-----------------тест deviation----------------------------
    copter_c = [0, 1, -1]
    prev = [-2-3, 6+4, -1-1]
    next = [-2, 6, -1]
    d = normal_point_calculation_xyz(copter_c, prev, next)
    #print("normal_point_calculation_xyz is ", d)
    print()
    d = get_deviation([], 0, copt_coord)
    d = get_deviation(goal_global_, 0, copt_coord)
    d = get_deviation(goal_global_, 0, copt_coord2)
    
    print("copt_coord_gps is ", copt_coord2, goal_global_)
    print("get_deviation is ", d)
    print("copter_coord_after_takeoff is ", copter_coord_after_takeoff)
    print()
    """
    #-----------------тест corridor----------------------------
    #copter_coord = [1, 1, 0]
    #prev = [0, 0, 0]
    #next = [0, 5, 0]
    #d = corridor_point_calculation_xyz(copter_coord, prev, next)
    d = get_corridor_point([], 0, copt_coord)
    print("get_corridor_point check 1 is ", d)
    print()
    d = get_corridor_point(goal_global_, 0, copt_coord)
    print("get_corridor_point check 2 is ", d)
    print()
    
    d = get_corridor_point(goal_global_, 0, copt_coord2)
    print("get_corridor_point check 2 is ", d)
    print()
    
    d = get_corridor_point(goal_global_, 0, copt_coord3)
    print("copt_coord is ", copt_coord3, goal_global_)
    print("get_corridor_point check 3 is ", d)
    print("copter_coord_after_takeoff is ", copter_coord_after_takeoff)
    print()
    
# TODO
# 1. Реализовать вылет за точку в deviation и corridor_point


# 
#TODO
#----------------------------------------------------------

"""
# Проверить proceed_coordinate_system()
Сейчас 
proceed_coordinate_system([Gps_x, Gps_y, Gps_z], [x1, y1, z1], [x2, y2, z2]])
    Proceeded
    copter_coord_xyz = [x2 - xc, y2 - yc, z2 - zc]
    return copter_coord_xyz
Нужно
proceed_coordinate_system([Gps_x, Gps_y, Gps_z], [x1, y1, z1], [x2, y2, z2]])
    Proceeded
    copter_coord_xyz = [x2 - xc, y2 - yc, z2 - zc]
    Чо-то там сделали
    copter_coord_xyz = [xc, yc, zc]
    return copter_coord_xyz
"""

#Мусорка
#----------------------------------------------------------
"""
# ВНИМАНИЕ!! эта функция не точна т.к. земля круглая, а gps мы преобразовываем позже
def gps_check(copter_coord, point_first, point_last):
    
    x2, y2, z2, xc, yc, zc
    directing_vector = [l1, l2, l3]

    Vector_Copter_Goal = [x2 - xc, y2, - yc, z2 - zc]

    Vector_C_G_S[0] = Vector_Copter_Goal[1] * directing_vector[2] - Vector_Copter_Goal[2] * directing_vector[1]
    Vector_C_G_S[1] = - ( Vector_Copter_Goal [0] * directing_vector[2] - Vector_Copter_Goal[2] * directing_vector[0] )
    Vector_C_G_S[2] = Vector_Copter_Goal[0] * directing_vector[1] - Vector_Copter_Goal[1] * directing_vector[0]

    directing_vector_length = math.hypot(directing_vector[0], directing_vector[1], directing_vector[2])

    Vector_C_G_S_length = math.hypot(Vector_C_G_S[0], Vector_C_G_S[1], Vector_C_G_S[2])
    deviation = Vector_C_G_S_length/directing_vector_length

    deviation = tpd.enu_vector(copter_coord_gps, point_first)

    # ... перевести в дельту координат

    Earth_radius = 6378.137; // Radius of earth in KM
    dLat = 0
    dLon = 0
    a = Math.sin(dLat/2) * Math.sin(dLat/2) +
    Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) *
    Math.sin(dLon/2) * Math.sin(dLon/2);
    var c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
    var d = R * c;
    d_meters = d * 1000; // meters

    return deviation
"""
