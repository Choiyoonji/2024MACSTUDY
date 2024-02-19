import pandas as pd
import numpy as np
import math
pi=math.pi
sin=math.sin
cos=math.cos

#DH parameter
th=np.array([0,0,0])*pi/180 #rad
a=np.array([0,-425,-392]) #mm
d=np.array([89,0,0]) #mm
al=np.array([90,0,0])*pi/180 #rad

T=[0 for i in range(3)]

for i in range(3):
  T[i]=np.array([[cos(th[i]), -sin(th[i])*cos(al[i]), sin(th[i])*sin(al[i]) , a[i]*cos(th[i])],
                [sin(th[i]), cos(th[i])*cos(al[i]) , -cos(th[i])*sin(al[i]), a[i]*sin(th[i])],
                [0         , sin(al[i])            , cos(al[i])            , d[i]           ],
                [0         ,0                      , 0                     , 1              ]])

#T02: T[0]*T[1]*T[2]
  T02=pd.DataFrame(T[0].dot(T[1]).dot(T[2]))
  
  print(T02)
