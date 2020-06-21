#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ============================================================================ #
import math
import weakref
import numpy as np
import matplotlib.pyplot as plt
import MazeLib  # calls $PYTHONPATH/MazeLib.so


class MazeDrawer:
    def __init__(self, maze, size=MazeLib.MAZE_SIZE):
        self.maze = maze
        self.wall_lines = []  # index is WallIndex
        self.paths = []
        # parameter
        line_width = 1
        background_color = 'k'
        label_color = 'w'
        # prepare figure
        figsize = size // 3
        plt.figure(figsize=(figsize, figsize))
        # make wall_lines
        for i in range(0, MazeLib.WallIndex.SIZE):
            w = MazeLib.WallIndex(i)
            p = w.getPosition()
            x, y = p.x, p.y
            if int(w.getDirection()) == MazeLib.Direction.East:
                x, y = [x+1/2, x+1/2], [y-1/2, y+1/2]
            else:
                x, y = [x-1/2, x+1/2], [y+1/2, y+1/2]
            wall_line, = plt.plot(x, y, 'r', lw=line_width)
            self.wall_lines.append(wall_line)
            self.draw_wall(maze, w)
        # show S and G
        for ps, t in [[[maze.getStart()], 'S'], [maze.getGoals(), 'G']]:
            for p in ps:
                plt.text(p.x, p.y, t, ha='center', va='center', c=label_color)
        for axis in ['top', 'bottom', 'left', 'right']:
            plt.gca().spines[axis].set_linewidth(line_width)
            plt.gca().spines[axis].set_color('r')
        plt.gca().set_aspect('equal')  # set the x and y axes to the same scale
        plt.gca().set_facecolor(background_color)
        plt.gcf().set_facecolor(background_color)
        plt.rcParams['savefig.facecolor'] = background_color
        plt.xticks(range(0, size+1, 1), color=label_color)
        plt.yticks(range(0, size+1, 1), color=label_color)
        plt.xlim([-1/2, size-1/2])
        plt.ylim([-1/2, size-1/2])
        plt.tight_layout()

    def draw_path(self, start, directions, color='y', diag_enabled=False):
        if diag_enabled:
            return self.draw_path_wall(start, directions, color)
        lines = []
        p = start
        for d in directions:
            p_next = p.next(d)
            x, y = [p.x, p_next.x], [p.y, p_next.y]
            line, = plt.plot(x, y, c=color)
            p = p_next
            lines.append(line)
        self.paths.append(lines)

    def draw_path_wall(self, start, directions, color='y'):
        p = start
        indexes = []
        for d in directions:
            indexes.append(MazeLib.WallIndex(p, d))
            p = p.next(d)
        lines = []
        for i in range(len(indexes)-1):
            w = indexes[i]
            p = w.getPosition()
            w_next = indexes[i+1]
            p_next = w_next.getPosition()
            if int(w.getDirection()) == MazeLib.Direction.East:
                x, y = p.x+1/2, p.y
            else:
                x, y = p.x, p.y+1/2
            if int(w_next.getDirection()) == MazeLib.Direction.East:
                x_next, y_next = p_next.x+1/2, p_next.y
            else:
                x_next, y_next = p_next.x, p_next.y+1/2
            x, y = [x, x_next], [y, y_next]
            line, = plt.plot(x, y, c=color)
            lines.append(line)
        self.paths.append(lines)

    def draw_wall(self, maze, w):
        if not w.isInsideOfField():
            return
        wall_line = self.wall_lines[w.getIndex()]
        if not maze.isKnown(w):
            wall_line.set_color('red')
            wall_line.set_linestyle(':')
        elif maze.isWall(w):
            wall_line.set_color('red')
            wall_line.set_linestyle('-')
        elif not maze.isWall(w):
            wall_line.set_color('gray')
            wall_line.set_linestyle(':')

    def attach_wall_toggle(self):
        plt.connect('button_press_event', self.button_press_event)

    def button_press_event(self, event):
        maze = self.maze
        wi = self.find_nearest_wall(event)
        maze.setWall(wi, not maze.isWall(wi))
        maze.setKnown(wi, True)
        self.draw_wall(maze, wi)
        self.update_path()
        plt.draw()

    def find_nearest_wall(self, event):
        x, y = event.xdata, event.ydata
        xf, xi = math.modf(x)
        yf, yi = math.modf(y)
        z, d = [0, MazeLib.Direction.East] if abs(
            xf-1/2) < abs(yf-1/2) else [1, MazeLib.Direction.North]
        if z == 0:
            x, y = int(round(x-1/2)), int(round(y))
        else:
            x, y = int(round(x)), int(round(y-1/2))
        wi = MazeLib.WallIndex(MazeLib.Position(x, y), d)
        return wi

    def update_path(self):
        for path in self.paths:
            while path:
                path.pop(0).remove()
        maze = self.maze
        # step_map
        step_map = MazeLib.StepMap()
        sd = step_map.calcShortestDirections(maze, simple=True)
        if not sd:
            print("Failed to Find any Path!")
        MazeLib.StepMap.appendStraightDirections(
            maze, sd, known_only=True, diag_enabled=False)
        self.draw_path(maze.getStart(), sd, color='b')
        # step_map_wall
        step_map_wall = MazeLib.StepMapWall()
        sd = step_map_wall.calcShortestDirections(maze, simple=True)
        sd = step_map_wall.convertWallIndexDirectionsToPositionDirections(sd)
        MazeLib.StepMap.appendStraightDirections(
            maze, sd, known_only=True, diag_enabled=True)
        self.draw_path_wall(maze.getStart(), sd, color='g')
        # step_map_slalom
        step_map = MazeLib.StepMapSlalom()
        for diag_enabled in [False, True]:
            sd = step_map.calcShortestDirections(
                maze, diag_enabled=diag_enabled)
            MazeLib.StepMap.appendStraightDirections(
                maze, sd, known_only=True, diag_enabled=diag_enabled)
            c = 'y' if diag_enabled else 'c'
            self.draw_path(maze.getStart(), sd, color=c,
                           diag_enabled=diag_enabled)


def plot_maze():
    # filepath = '../mazedata/data/32MM2019HX.maze'
    # filepath = '../mazedata/data/32MM2018HX.maze'
    filepath = '../mazedata/data/32MM2015HX.maze'
    # filepath = '../mazedata/data/16MM2019CX.maze'
    # filepath = '../mazedata/data/16MM2019H_semi.maze'
    maze = MazeLib.Maze()
    if not maze.parse(filepath):
        print("Failed to Parse Maze File: ", filepath)
    size = maze.getMaxX() + 1

    maze_drawer = MazeDrawer(maze, size)
    maze_drawer.attach_wall_toggle()
    maze_drawer.update_path()

    plt.show()


plot_maze()
