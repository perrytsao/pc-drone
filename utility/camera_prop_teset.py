# -*- coding: utf-8 -*-
"""
Created on Sat Mar 12 15:32:28 2016

@author: perrytsao
"""

fourcc = vc.get(cv2.CAP_PROP_FOURCC)

fourcc = int(fourcc)

fourcc = chr((fourcc & 0XFF)) + chr((fourcc & 0XFF00) >> 8)+ chr((fourcc & 0XFF0000) >> 16)+ chr((fourcc & 0XFF000000) >> 24) + '\0'

#fourcc = cv2.VideoWriter_fourcc(fourcc[0],fourcc[1],fourcc[2],fourcc[3])