import os
import matplotlib.pyplot as plt
from matplotlib import rcParams
import numpy as np
import math


PurpPosition = [-1.7, -0.001]
MyPosition = [-0.726, -1.127]
PurpVect = [PurpPosition[0] - MyPosition[0], PurpPosition[1] - MyPosition[1]]
OxVect = [-1, 0]

'''dot = OxVect[0]*OxVect[1] + PurpVect[0]*PurpVect[1]
det = OxVect[0]*PurpVect[1] - PurpVect[0]*OxVect[1]
angle = math.atan2(det, dot)'''
TempCosDiv = math.sqrt((PurpVect[0])**2+(PurpVect[1])**2)*math.sqrt((OxVect[0])**2+(OxVect[1])**2);
AngleCos = (PurpVect[0]*OxVect[0]+PurpVect[1]*OxVect[1])/TempCosDiv
rotationVal = 180*AngleCos/3.14
if()
plt.plot([-10, 10], [0, 0], 'black')
plt.plot([0, 0], [-10, 10], 'black')
plt.plot(PurpVect, 'blue')
plt.plot(OxVect)
print(rotationVal, 'blue')
print(PurpVect[0], PurpVect[1])
plt.show()


