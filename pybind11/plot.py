#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ============================================================================ #
import math
import numpy as np
import matplotlib.pyplot as plt
import MazeLib  # calls $PYTHONPATH/MazeLib.so


class MazeDrawer:
    """
    A tool to draw a maze
    """

    def __init__(self, maze):
        """
        construct a maze editor object with a maze object
        """
        self.maze = maze  # maze object

    def draw_maze(self):
        maze = self.maze
        size = max(maze.getMaxX(), maze.getMaxY())
        # print walls
        for i in range(size+1):
            for j in range(size):
                # +---+---+
                if maze.isWall(i, j, MazeLib.Direction.South):
                    self.draw_wall(maze, i, j, MazeLib.Direction.South)
                else:
                    self.draw_wall(maze, i, j, MazeLib.Direction.South,
                                   ':', color='gray')
                # |   |   |
                if maze.isWall(j, i, MazeLib.Direction.West):
                    self.draw_wall(maze, j, i, MazeLib.Direction.West)
                else:
                    self.draw_wall(
                        maze, j, i, MazeLib.Direction.West, ':', color='gray')
        # print start and goal character
        for ps, t in [[[maze.getStart()], 'S'], [maze.getGoals(), 'G']]:
            for p in ps:
                plt.text(p.x, p.y, t, ha='center', va='center')
        plt.gca().set_aspect('equal')  # set the x and y axes to the same scale
        plt.xticks(range(0, size+1, 1))
        plt.yticks(range(0, size+1, 1))
        plt.xlim([-1/2, size-1/2])
        plt.ylim([-1/2, size-1/2])
        plt.tight_layout()

    @staticmethod
    def draw_wall(maze, x, y, d, fmt='k', **kwargs):
        w = MazeLib.WallIndex(MazeLib.Position(x, y), d)
        p = w.getPosition()
        x, y = p.x, p.y
        if int(w.getDirection()) == int(MazeLib.Direction.East):
            x, y = [x+1/2, x+1/2], [y-1/2, y+1/2]
        else:
            x, y = [x-1/2, x+1/2], [y+1/2, y+1/2]
        plt.plot(x, y, 'w', lw=2)  # erase wall
        plt.plot(x, y, fmt, **kwargs)  # draw new wall
        plt.plot(x, y, 'k.', lw=1)  # draw pillars

    def attach_wall_toggle(self):
        plt.connect('button_press_event', self.button_press_event)

    def button_press_event(self, event):
        maze = self.maze
        x, y = event.xdata, event.ydata
        xf, xi = math.modf(x)
        yf, yi = math.modf(y)
        z, d = [0, MazeLib.Direction.East] if abs(
            xf-1/2) < abs(yf-1/2) else [1, MazeLib.Direction.North]
        if z == 0:
            x, y = int(round(x-1/2)), int(round(y))
        else:
            x, y = int(round(x)), int(round(y-1/2))
        w = not maze.isWall(x, y, d)
        c = 'r' if w else 'r:'
        maze.setWall(x, y, d, w)
        maze.setKnown(x, y, d, True)
        self.draw_wall(maze, x, y, d, c)
        plt.draw()


def plot_maze():
    # filepath = '../mazedata/data/32MM2019HX.maze'
    filepath = '../mazedata/data/16MM2019CX.maze'
    maze = MazeLib.Maze()
    if not maze.parse(filepath):
        print("Failed to Parse Maze File: ", filepath)
    size = max(maze.getMaxX(), maze.getMaxY())
    maze.print(size)

    figsize = size // 2
    fig = plt.figure(figsize=(figsize, figsize))
    maze_drawer = MazeDrawer(maze)
    maze_drawer.draw_maze()
    maze_drawer.attach_wall_toggle()
    plt.show()


plot_maze()
