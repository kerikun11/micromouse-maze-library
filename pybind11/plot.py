#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ============================================================================ #
import numpy as np
import matplotlib.pyplot as plt

import MazeLib  # calls $PYTHONPATH/MazeLib.so


def plot_maze():
    for d in MazeLib.Direction.getAlong4():
        print(d)
    # plt.show()
    print(MazeLib.Direction(9))
    print(MazeLib.Direction.East)
    print(MazeLib.Direction.AbsoluteDirection.East)
    # print(MazeLib.East)
    print(MazeLib.Position.SIZE)


plot_maze()
