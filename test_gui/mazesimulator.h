#ifndef MAZESIMULATOR_H
#define MAZESIMULATOR_H

#include <QObject>
#include <QWidget>

#include "Maze.h"

class MazeSimulator : public QObject
{
    Q_OBJECT
public:
    explicit MazeSimulator(QObject *parent = nullptr);
    void drawMaze(const MazeLib::Maze& maze) const {
    }

signals:

public slots:
};

#endif // MAZESIMULATOR_H
