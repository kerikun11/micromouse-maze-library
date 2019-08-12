#ifndef MAZEDRAWER_H
#define MAZEDRAWER_H

#include <QGraphicsScene>
#include <QGraphicsItem>
#include "Maze.h"

class MazeDrawer
{
public:
    MazeDrawer();

    void clear(QGraphicsScene* scene){
        scene->setBackgroundBrush(Qt::black);
        int w = wall_unit_px;
        int s = MazeLib::MAZE_SIZE;
        /* Print Line Number */
        for(int i=0; i<MazeLib::MAZE_SIZE; ++i) {
            scene->addText(QString::number(i))->setPos((i+1+0.25)*w, (s+1)*w);
            scene->addText(QString::number(i))->setPos(0, (s-i)*w);
        }
        /* Print Cell Line */
        QPen pen;
        pen.setColor(Qt::gray);
        pen.setStyle(Qt::DotLine);
        for(int i=0; i<MazeLib::MAZE_SIZE + 1; ++i) {
            scene->addLine(cell2posX(i), cell2posY(0), cell2posX(i), cell2posY(s), pen);
            scene->addLine(cell2posX(0), cell2posY(i), cell2posX(s), cell2posY(i), pen);
        }
    }
    void draw(QGraphicsScene* scene, const MazeLib::Maze& maze) {
        /* Print Walls */
        QPen pen;
        pen.setColor(Qt::red);
        for(int x=0; x<MazeLib::MAZE_SIZE + 1; ++x)
            for(int y=0; y<MazeLib::MAZE_SIZE + 1; ++y) {
                if(y!=MazeLib::MAZE_SIZE && maze.isWall(x, y, MazeLib::Dir::West))
                    scene->addLine(cell2posX(x), cell2posY(y), cell2posX(x), cell2posY(y+1), pen);
                if(x!=MazeLib::MAZE_SIZE && maze.isWall(x, y, MazeLib::Dir::South))
                    scene->addLine(cell2posX(x), cell2posY(y), cell2posX(x+1), cell2posY(y), pen);
            }
    }

private:
    int wall_unit_px = 28;

    int cell2posX(int x) const {
        int w = wall_unit_px;
        int s = MazeLib::MAZE_SIZE;
        return w+x*w;
    }
    int cell2posY(int y) const {
        int w = wall_unit_px;
        int s = MazeLib::MAZE_SIZE;
        return (s-y+1)*w;
    }
};

#endif // MAZEDRAWER_H
