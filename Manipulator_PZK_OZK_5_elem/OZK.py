import numpy as np
import math


def OZK_Task(mml, mrl1, mrl2):
    # Начальные углы (всегда константы)
    alpha = math.pi / 4
    betta = math.pi / 4

    print("введите координаты схвата")
    X = float(input())
    Y = float(input())
    Z = float(input())
    vector_1_5 = np.array([X, Y, Z])
    # Матрицы поворотов
    zMatrix = np.array([[math.cos(alpha), -math.sin(alpha), 0],
               [math.sin(alpha), math.cos(alpha), 0],
               [0, 0, 1]])
    xMatrix = np.array([[math.cos(betta), 0, math.sin(betta)],
               [0, 1, 0],
               [-math.sin(betta), 0, math.cos(betta)]])
    yMatrix = np.array([[1, 0, 0],
               [0, math.cos(betta), -math.sin(betta)],
               [0, math.sin(betta), math.cos(betta)]])

    rMatrix = yMatrix * xMatrix

    vector_3_5 = vector_1_5 - np.array([mrl2 * math.cos(alpha) * math.cos(betta),
                  - mrl1 * math.sin(alpha) * math.cos(betta),
                  mrl1 + mrl2 * math.sin(betta)])
    alphaRot = math.acos(rMatrix[1][1])
    bettaRot = math.acos(rMatrix[2][2])

    print("результаты линейного перемещения(обобщ коорд) - ", vector_3_5)
    print("результаты вращательного перемещения(обобщ коорд) - ", alphaRot, " ", bettaRot)

    return 0


# Длина звеньев
manipulatorMotionLinks = np.array([3, 3, 3])
manipulatorRotationLink1 = 1.5
manipulatorRotationLink2 = 1.5

OZKResult = OZK_Task(manipulatorMotionLinks, manipulatorRotationLink1, manipulatorRotationLink2)

