
import numpy as np
import math


def PZKfunc(q):
    x1, y1, z1, alp, be = tuple(q)
    if (x1 > length1) & (x2 > length2) & (x3 > length3):
        print("говно собачье, выключай да")
        return -1
    x = x1 + length[0] + length[4] * math.cos(alp) * math.cos(be)
    y = y1 + length[1] + length[4] * math.sin(alp) * math.sin(be)
    z = z1 + length[2] + length[3] + length[4] * math.sin(be)
    return x, y, z


# длина звениев
length = [3, 3, 3, 2, 2]
# перемещение звеньев
q = [0, 0, 0, 0, 0]

#pzk

for i in range(0, length.size()) :
    print("звено ", i, " - ", length[i], "/n")

for i in range(0, q.size()) :
    print("введите перемещение звена i")
    q[i] = input()

try:
    X, Y, Z = PZKfunc(q)
    print("ну собсна вот координаты, хрыщ", X, " ", Y)

except:
    print("идиот")


"""
Ralp = np.array([[math.cos(alp1), -math.sin(alp1), 0],
                     [math.sin(alp1), math.cos(alp1), 0],
                     [0, 0, 1]])
    Rbe = np.array([[1, 0, 0],
                    [0, math.cos(be1), -math.sin(be1)],
                    [0, math.sin(be1), math.cos(be1)]])
    R = np.matmul(Rbe, Ralp)

    XYZVect = np.array([x1],
                       [y1],
                       [z1])
"""