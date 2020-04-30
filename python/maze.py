#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ============================================================================ #
import sys
import math
import numpy as np
import matplotlib.pyplot as plt


class Maze:
    """
    a maze class includes wall data, start indexes, goal indexes
    """
    # constants
    East, North, West, South = range(4)

    def __init__(self, size=32):
        """
        construct a maze object with a maze size
        """
        self.size = size  # number of cells on one side of the maze
        self.index_size = size * size * 2  # the number of walls to save: x * y * z
        self.walls = np.zeros(self.index_size, dtype=bool)  # wall flags
        self.start = []
        self.goals = []

    @classmethod
    def uniquify(cls, x, y, d):
        """
        returns a unique coordinates of a wall without redundancy on both sides of the wall
        """
        if d == cls.East:
            x, y, z, d = x, y, 0, cls.East
        elif d == cls.North:
            x, y, z, d = x, y, 1, cls.North
        elif d == cls.West:
            x, y, z, d = x-1, y, 0, cls.East
        elif d == cls.South:
            x, y, z, d = x, y-1, 1, cls.North
        return x, y, z, d

    def index(self, x, y, z):
        """
        get a unique id of a wall inside of the maze
        """
        if not self.is_inside_of_field(x, y, z):
            raise ValueError("out of field!")
        return z * self.size * self.size + y * self.size + x

    def is_inside_of_field(self, x, y, z):
        """
        determine if the wall is inside of the field
        """
        s = self.size
        return x >= 0 and y >= 0 and x < s+z-1 and y < s-z

    def wall(self, x, y, d, update=None):
        """
        get wall or update wall
        """
        x, y, z, d = self.uniquify(x, y, d)
        if not self.is_inside_of_field(x, y, z):
            return True
        i = self.index(x, y, z)
        if update != None:
            self.walls[i] = update
        return self.walls[i]

    def __str__(self):
        """
        show information of the maze
        """
        return f'size: {self.size}x{self.size}' + '\n' + \
            'start: ' + ', '.join([f'({x}, {y})' for x, y in self.start]) + '\n' + \
            'goals: ' + ', '.join([f'({x}, {y})' for x, y in self.goals])

    def generate_maze_string(self):
        """
        generate a string to be saved in text format
        """
        res = ''
        for y in reversed(range(-1, self.size)):
            # +---+---+
            res += '+'
            for x in range(self.size):
                res += '---' if maze.wall(x, y, Maze.North) else '   '
                res += '+'
            res += '\n'
            # |   |   |
            if y == -1:
                break
            res += '|'
            for x in range(self.size):
                if [x, y] in maze.start:
                    res += ' S '
                elif [x, y] in maze.goals:
                    res += ' G '
                else:
                    res += '   '
                res += '|' if maze.wall(x, y, Maze.East) else ' '
            res += '\n'
        return res

    @staticmethod
    def parse(file):
        """
        parse maze string and construct maze object
        """
        lines = file.readlines()
        maze = Maze(len(lines)//2)
        # print(f'maze size: {maze.size}x{maze.size}')
        for i, line in enumerate(reversed(lines)):
            line = line.rstrip()  # remove \n
            # +---+---+
            if i % 2 == 0:
                for j, c in enumerate(line[2:: 4]):
                    if c == '-':
                        maze.wall(j, i//2, Maze.South, True)
            # |   |   |
            else:
                for j, c in enumerate(line[0:: 4]):
                    if c == '|':
                        maze.wall(j, i//2, Maze.West, True)
                for j, c in enumerate(line[2:: 4]):
                    if c == 'S':
                        # print(f"S: ({j},{i//2})")
                        maze.start.append([j, i//2])
                    if c == 'G':
                        # print(f"G: ({j},{i//2})")
                        maze.goals.append([j, i//2])
        return maze


class MazePainter:
    @staticmethod
    def draw_wall(maze, x, y, d, fmt='k', **kwargs):
        x, y, z, d = Maze.uniquify(x, y, d)
        if z == 0:
            x, y = [x+1/2, x+1/2], [y-1/2, y+1/2]
        else:
            x, y = [x-1/2, x+1/2], [y+1/2, y+1/2]
        plt.plot(x, y, 'w', lw=2)
        plt.plot(x, y, fmt, **kwargs)
        plt.plot(x, y, 'k.')

    @classmethod
    def draw_maze(cls, maze):
        for i in range(maze.size+1):
            for j in range(maze.size):
                # +---+---+
                if maze.wall(i, j, Maze.South):
                    cls.draw_wall(maze, i, j, Maze.South)
                else:
                    cls.draw_wall(maze, i, j, Maze.South, ':', color='gray')
                # |   |   |
                if maze.wall(j, i, Maze.West):
                    cls.draw_wall(maze, j, i, Maze.West)
                else:
                    cls.draw_wall(maze, j, i, Maze.West, ':', color='gray')
        for ps, t in [[maze.start, 'S'], [maze.goals, 'G']]:
            for p in ps:
                plt.text(p[0], p[1], t, ha='center', va='center')
        plt.axes().set_aspect('equal')  # set the x and y axes to the same scale
        plt.xticks(range(0, maze.size+1, 1))
        plt.yticks(range(0, maze.size+1, 1))
        plt.xlim([-1/2, maze.size-1/2])
        plt.ylim([-1/2, maze.size-1/2])
        plt.tight_layout()


# ============================================================================ #
if __name__ == "__main__":
    filepath = './mazedata/32MM2019HX.maze'
    # filepath = './mazedata/16MM2019CX.maze'
    # filepath = './mazedata/09MM2019C_Cheese_cand.maze'

    # overwrite with commandline argument
    if len(sys.argv) > 1:
        filepath = sys.argv[1]

    # read file
    with open(filepath, 'r') as file:
        maze = Maze.parse(file)
    print(maze)
    print(maze.generate_maze_string())

    # setup maze modifier
    def button_press_event(event):
        x, y = event.xdata, event.ydata
        xf, xi = math.modf(x)
        yf, yi = math.modf(y)
        z, d = [0, Maze.East] if abs(xf-1/2) < abs(yf-1/2) else [1, Maze.North]
        if z == 0:
            x, y = int(round(x-1/2)), int(round(y))
        else:
            x, y = int(round(x)), int(round(y-1/2))
        w = not maze.wall(x, y, d)
        c = 'r' if w else 'r:'
        maze.wall(x, y, d, w)
        MazePainter.draw_wall(maze, x, y, d, c)
        plt.draw()
        print(maze.generate_maze_string())
    fig = plt.figure(figsize=(10, 10))
    MazePainter.draw_maze(maze)
    plt.connect('button_press_event', button_press_event)
    plt.show()

    # save modified maze
    with open('./build/modified_maze.maze', 'w') as file:
        file.write(maze.generate_maze_string())
