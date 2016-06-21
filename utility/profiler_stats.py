# -*- coding: utf-8 -*-
"""
Created on Sun May 29 04:03:52 2016

@author: perrytsao
"""

import pstats
p = pstats.Stats('runstats.txt')
#p.strip_dirs().sort_stats(-1).print_stats()
p.sort_stats('cumulative').print_stats(10)