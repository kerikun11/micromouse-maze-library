#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ============================================================================ #
from maze_drawer import MazeDrawer  # calls maze_drawer.py
import matplotlib.pyplot as plt
import MazeLib  # calls $PYTHONPATH/MazeLib.so
import sys


def plot_maze():
    filepath = '../mazedata/data/32MM2022HX.maze'
    # filepath = '../mazedata/data/16MM2019CX.maze'

    # if argument exists, the filepath is overridden.
    if len(sys.argv) >= 2:
        filepath = sys.argv[1]

    maze = MazeLib.Maze()
    if not maze.parse(filepath):
        print("Failed to Parse Maze File: ", filepath)
    size = maze.getMaxX() + 1

    maze_drawer = MazeDrawer(maze, size)
    maze_drawer.attach_wall_toggle()
    maze_drawer.update_path()

    plt.show()


plot_maze()
