#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ============================================================================ #
import numpy as np
import matplotlib.pyplot as plt

import maze as MazeLib  # calls build/maze.so


def plot_maze():
    for d in MazeLib.Direction.getAlong4():
        print(d)
    # plt.show()
    print(MazeLib.Direction(9))
    print(MazeLib.AbsoluteDirection.East)
    # print(MazeLib.East)


plot_maze()
