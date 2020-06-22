#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ============================================================================ #
from maze_drawer import MazeDrawer  # calls maze_drawer.py
import matplotlib.pyplot as plt
import MazeLib  # calls $PYTHONPATH/MazeLib.so


def plot_maze():
    # filepath = '../mazedata/data/32MM2019HX.maze'
    # filepath = '../mazedata/data/32MM2018HX.maze'
    # filepath = '../mazedata/data/32MM2015HX.maze'
    # filepath = '../mazedata/data/16MM2019CX.maze'
    filepath = '../mazedata/data/16MM2019H_semi.maze'
    maze = MazeLib.Maze()
    if not maze.parse(filepath):
        print("Failed to Parse Maze File: ", filepath)
    size = maze.getMaxX() + 1

    maze_drawer = MazeDrawer(maze, size)
    maze_drawer.attach_wall_toggle()
    maze_drawer.update_path()

    plt.show()


plot_maze()
