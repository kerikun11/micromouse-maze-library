#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ============================================================================ #
import numpy as np
import matplotlib.pyplot as plt

from MazeLib import *  # calls $PYTHONPATH/MazeLib.so
import MazeLib  # calls $PYTHONPATH/MazeLib.so


def plot_maze():
    for d in MazeLib.Direction.getAlong4():
        print(d)
    # plt.show()
    print(MazeLib.Direction(9))
    print(MazeLib.Direction.East)
    print(MazeLib.Direction(MazeLib.Direction.East))
    print(MazeLib.Direction.AbsoluteDirection.East)
    # print(MazeLib.East)
    print(MazeLib.Position.SIZE)
    print(int(Direction.East))

    maze = MazeLib.Maze()
    maze.parse('../mazedata/data/32MM2019HX.maze')
    maze.print()


plot_maze()
