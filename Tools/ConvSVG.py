import math
import cmath
import numpy as np
from svgpathtools import svg2paths, wsvg

paths, attributes = svg2paths('Tsukuba.svg')
redpath = paths[0]
redpath_attribs = attributes[0]
myPath = np.empty((0,3),float)
myPathDiff = np.empty((0,3),float)

for i in range(len(redpath)):
    if str(redpath[i].__class__) == "<class 'svgpathtools.path.CubicBezier'>" :
            myPath = np.append(myPath,np.array([[redpath[i].end.real, -redpath[i].end.imag, -cmath.phase(redpath[i].end-redpath[i].control2).real]]),axis=0)
    #if(i == 0) :
        #print(str(redpath[i][0].real) + "," + str(redpath[i][0].imag) + "," + str(math.atan2(redpath[i][1].imag, redpath[i][1].real)))
    #else:
        #print(str(redpath[i-1][3].real+redpath[i][0].real) + "," + str(redpath[i-1][3].imag+redpath[i][0].imag) + "," + str(math.atan2(redpath[i-1][2].imag, redpath[i-1][2].real) + math.atan2(redpath[i][1].imag, redpath[i][1].real)))
myPath[:,0] = myPath[:,0] - min(myPath[:,0])#
myPath[:,1] = myPath[:,1] - min(myPath[:,1])#
k = 32767/max(abs(myPath[:,0:1]))
myPath[:,0] *= k
myPath[:,1] *= k
myPath[:,2] *= 10000
#myPath = np.round(myPath)
#myPathDiff = np.diff(myPath,axis=0)
np.savetxt('course.csv',myPath,fmt="{%d,%d,%d}",delimiter='',newline = ',\n')
np.savetxt('devcourse.csv',myPath,fmt="%d",delimiter=',')
