import numpy as np
import math

# длина звениев
length = [3, 3, 3, 2, 2]


# pzk
def PZKfunc(q):
    x1, y1, z1, alp, be = tuple(q)
    if (x1 > length[1]) | (y1 > length[2]) | (z1 > length[3]):
        return -1
    x = x1 + length[0] + length[4] * math.cos(alp) * math.cos(be)
    y = y1 + length[1] + length[4] * math.sin(alp) * math.sin(be)
    z = z1 + length[2] + length[3] + length[4] * math.sin(be)
    return x, y, z


# перемещение звеньев
q = [0, 0, 0, 0, 0]

for i in range(0, 5):
    print("звено ", i, " - ", length[i], "/n")

for i in range(0, 5):
    print("введите перемещение звена i")
    q[i] = int(input())

try:
    X, Y, Z = PZKfunc(q)
    print("координаты", X, " ", Y, " ", Z)

except:
    print("Одно из звеньев больше, чем ")