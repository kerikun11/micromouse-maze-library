#ifndef MAZESIMULATOR_H
#define MAZESIMULATOR_H

#include <QEventLoop>
#include <QGraphicsItem>
#include <QGraphicsScene>
#include <QMainWindow>
#include <QTimer>
#include <sstream>

#include "Maze.h"
#include "RobotBase.h"
#include "ui_mainwindow.h"

using namespace MazeLib;

namespace Ui {
class MainWindow;
}

/**
 * @brief The MazeSimulator class
 */
class MazeSimulator : public MazeLib::RobotBase {
public:
  MazeSimulator(Ui::MainWindow *ui, QGraphicsScene *scene)
      : RobotBase(maze), ui(ui), scene(scene) {
    loop->connect(timer, SIGNAL(timeout()), loop, SLOT(quit()));
  }

  void reset() {
    maze.reset();
    RobotBase::reset();
  }
  void clear() {
    /* set Background Color */
    scene->clear();
    scene->setBackgroundBrush(Qt::black);
    /* Print Line Number */
    int w = wall_unit_px;
    int s = MazeLib::MAZE_SIZE;
    for (int i = 0; i < MazeLib::MAZE_SIZE; ++i) {
      QFont font;
      font.setPointSize(font_size);
      scene->addText(QString::number(i), font)->setPos((i + 0.25) * w, s * w);
      scene->addText(QString::number(i), font)->setPos(-w, (s - i - 1) * w);
    }
    /* Print Cell Line */
    //        QPen pen;
    //        pen.setColor(Qt::gray);
    //        pen.setStyle(Qt::DotLine);
    //        for(int i=0; i<MazeLib::MAZE_SIZE + 1; ++i) {
    //            scene->addLine(cell2posX(i), cell2posY(0), cell2posX(i),
    //            cell2posY(s), pen); scene->addLine(cell2posX(0), cell2posY(i),
    //            cell2posX(s), cell2posY(i), pen);
    //        }
  }
  void drawMaze(const Maze &maze) {
    /* Print Walls */
    for (int x = 0; x < MazeLib::MAZE_SIZE + 1; ++x)
      for (int y = 0; y < MazeLib::MAZE_SIZE + 1; ++y) {
        for (const auto d :
             {MazeLib::Direction::West, MazeLib::Direction::South}) {
          /* skip when it's out of the field */
          if (x == MazeLib::MAZE_SIZE && d == MazeLib::Direction::South)
            continue;
          if (y == MazeLib::MAZE_SIZE && d == MazeLib::Direction::West)
            continue;
          /* Draw Wall */
          QPen pen(Qt::red);
          if (maze.isKnown(x, y, d)) {
            if (maze.isWall(x, y, d))
              pen.setStyle(Qt::SolidLine), pen.setWidth(pillar_px);
            else
              pen.setStyle(Qt::DotLine), pen.setColor(Qt::gray);
          } else {
            if (maze.isWall(x, y, d))
              pen.setStyle(Qt::DashDotLine);
            else
              pen.setStyle(Qt::DotLine);
          }
          addWall(scene, MazeLib::Pose{MazeLib::Position(x, y), d}, pen);
        }
      }
    /* start & goal */
    //    QFont font;
    //    font.setPointSize(font_size);
    //    const int w = wall_unit_px;
    //    const int s = MazeLib::MAZE_SIZE;
    //    const auto p = maze.getStart();
    //    scene->addText("S", font)->setPos((p.x + 0.2) * w, (s - p.y - 0.95) *
    //    w); for(const auto p: maze.getGoals())
    //        scene->addText("G", font)->setPos((p.x + 0.2) * w, (s - p.y -
    //        0.95) * w);
  }
  void drawStep(const StepMap &map) {
    for (int x = 0; x < MazeLib::MAZE_SIZE; ++x)
      for (int y = 0; y < MazeLib::MAZE_SIZE; ++y) {
        QFont font;
        // font.setPointSize(5);
        font.setPointSize(font_size);
        scene
            ->addText(QString::number(std::min((int)map.getStep(x, y), 999)),
                      font)
            ->setPos(cell2posX(x), cell2posY(y + 1));
        /* Draw Wall */
        QPen pen(Qt::red);
      }
  }
  void drawPose(const Pose &pose) {
    /* Draw Machine */
    const auto p = pose.p;
    const auto d = pose.d;
    QPolygon pol;
    pol << QPoint(0, wall_unit_px / 6) << QPoint(0, -wall_unit_px / 6)
        << QPoint(wall_unit_px / 4, 0);
    pol.translate(-QPoint(wall_unit_px * 2 / 3, 0));
    QMatrix mat;
    mat.rotate(-45 * d);
    pol = mat.map(pol);
    pol.translate(QPoint(cell2posX(p.x) + wall_unit_px / 2,
                         cell2posY(p.y) - wall_unit_px / 2));
    scene->addPolygon(pol, QPen(Qt::yellow), QBrush(Qt::yellow));
  }
  bool drawShortest(const Maze &maze, const bool diag_enabled,
                    const StepMapSlalom::EdgeCost &edge_cost = {}) {
    Maze maze_tmp = maze;
    SearchAlgorithm sa(maze_tmp);
    Directions dirs;
    if (!sa.calcShortestDirections(dirs, diag_enabled, edge_cost))
      return false;
    auto p = maze.getStart();
    for (size_t i = 0; i < dirs.size(); ++i) {
      if (diag_enabled && i == dirs.size() - 1)
        break;
      const auto d = dirs[i];
      const auto next_p = p.next(d);
      const auto next_d = diag_enabled ? dirs[i + 1] : d;
      QPen pen(Qt::yellow);
      QPoint offset(line_px / 2, line_px / 2);
      if (!diag_enabled) {
        pen.setColor(Qt::cyan);
        offset = -offset;
      }
      QPoint p1 = getGraphicPointByPose(Pose(p, d), diag_enabled) + offset;
      QPoint p2 =
          getGraphicPointByPose(Pose(next_p, next_d), diag_enabled) + offset;
      pen.setWidth(line_px);
      scene->addLine(p1.x(), p1.y(), p2.x(), p2.y(), pen);
      p = next_p;
    }
    return true;
  }
  bool drawShortestStepMap(const Maze &maze, const bool simple) {
    Maze maze_tmp = maze;
    const bool known_only = 0;
    const bool diag_enabled = false;
    StepMap map;
    Directions dirs = map.calcShortestDirections(maze, known_only, simple);
    if (dirs.empty())
      return false;
    StepMap::appendStraightDirections(maze, dirs, known_only, diag_enabled);
    auto p = maze.getStart();
    for (size_t i = 0; i < dirs.size(); ++i) {
      if (diag_enabled && i == dirs.size() - 1)
        break;
      const auto d = dirs[i];
      const auto next_p = p.next(d);
      const auto next_d = diag_enabled ? dirs[i + 1] : d;
      QPoint offset(-line_px, -line_px);
      QPoint p1 = getGraphicPointByPose(Pose(p, d), diag_enabled) + offset;
      QPoint p2 =
          getGraphicPointByPose(Pose(next_p, next_d), diag_enabled) + offset;
      QPen pen(Qt::blue);
      pen.setWidth(line_px);
      scene->addLine(p1.x(), p1.y(), p2.x(), p2.y(), pen);
      p = next_p;
    }
    return true;
  }
  bool drawShortestStepMapWall(const Maze &maze, const bool simple) {
    Maze maze_tmp = maze;
    const bool known_only = 0;
    const bool diag_enabled = true;
    StepMapWall map;
    Directions dirs = map.calcShortestDirections(maze, known_only, simple);
    if (dirs.empty())
      return false;
    map.appendStraightDirections(maze, dirs);
    auto p = WallIndex(0, 0, 1);
    for (size_t i = 0; i < dirs.size(); ++i) {
      const auto d = dirs[i];
      const auto next_p = p.next(d);
      QPoint offset(line_px, line_px);
      QPoint p1 = getGraphicPointByPose(Pose(p.getPosition(), p.getDirection()),
                                        diag_enabled) +
                  offset;
      QPoint p2 =
          getGraphicPointByPose(
              Pose(next_p.getPosition(), next_p.getDirection()), diag_enabled) +
          offset;
      QPen pen(Qt::magenta);
      pen.setWidth(line_px);
      scene->addLine(p1.x(), p1.y(), p2.x(), p2.y(), pen);
      p = next_p;
    }
    return true;
  }

  const Maze &getMazeTarget() { return maze_target; }
  void setMazeTarget(const Maze &maze) { maze_target = maze; }
  void drawStatus() {
    std::stringstream ss;
    ss << "State: " << SearchAlgorithm::getStateString(getState());
    ss << "\t";
    ss << "Pos: " << getCurrentPose();
    ui->statusBar->showMessage(ss.str().c_str());
  }
  void draw() {
    clear();
    drawMaze(maze);
    drawStep(getSearchAlgorithm().getStepMap());
    drawPose(getCurrentPose());
    drawStatus();
  }

  void toggle(const int ms = 100) {
    if (timer->isActive()) {
      timer->stop();
    } else {
      timer->start(ms);
    }
  }
  void next(int n = 1) {
    for (int i = 0; i < n; ++i)
      loop->exit();
  }

protected:
  Maze maze;
  Maze maze_target;

  virtual void senseWalls(bool &left, bool &front, bool &right) override {
    const auto &pose = getCurrentPose();
    left = !maze_target.canGo(pose.p, pose.d + Direction::Left);
    front = !maze_target.canGo(pose.p, pose.d + Direction::Front);
    right = !maze_target.canGo(pose.p, pose.d + Direction::Right);
#if 0
    /* 前1区画先の壁を読める場合 */
    if (!front)
      updateWall(current_pose.p.next(current_pose.d), current_pose.d,
                 !maze_target.canGo(pose.p.next(pose.d), pose.d));
#endif
  }
  virtual void calcNextDirectionsPreCallback() override {}
  virtual void calcNextDirectionsPostCallback(
      SearchAlgorithm::State prevState __attribute__((unused)),
      SearchAlgorithm::State newState __attribute__((unused))) override {
    if (newState == prevState)
      return;
    /* State Change has occurred */
  }
  virtual void discrepancyWithKnownWall() override {
    if (getState() != SearchAlgorithm::IDENTIFYING_POSITION) {
      printInfo();
      std::cout
          << "There was a discrepancy with known information! CurrentPose:\t"
          << getCurrentPose() << std::endl;
    }
  }
  virtual void
  queueAction(const SearchAction action __attribute_used__) override {
    /* draw */
    draw();
    /* block */
    int code = loop->exec();
    if (code < 0)
      return;
    /* release */
  }

private:
  QEventLoop *loop = new QEventLoop();
  QTimer *timer = new QTimer();
  Ui::MainWindow *ui;
  QGraphicsScene *scene;
  int wall_unit_px = 24 * 32 / MAZE_SIZE + 4;
  int pillar_px = 2 * 32 / MAZE_SIZE;
  int line_px = pillar_px;
  int wall_px = wall_unit_px - pillar_px;
  int font_size = 10 * 32 / MAZE_SIZE;

  QPoint getGraphicPointByPose(const Pose &pose,
                               const bool on_the_wall = false) {
    const auto p = pose.p;
    QPoint p1;
    if (on_the_wall) {
      p1 = QPoint(wall_unit_px / 2, 0);
      QMatrix m1;
      m1.rotate(-45 * pose.d);
      p1 = m1.map(p1);
    }
    return QPoint(p1.x() + cell2posX(p.x) + wall_unit_px / 2,
                  p1.y() + cell2posY(p.y) - wall_unit_px / 2);
  }
  QGraphicsItem *addWall(QGraphicsScene *scene, MazeLib::Pose pose,
                         const QPen &pen) {
    int x = pose.p.x;
    int y = pose.p.y;
    MazeLib::Direction d = pose.d;
    switch (d) {
    case MazeLib::Direction::East:
      return scene->addLine(cell2posX(x + 1), cell2posY(y), cell2posX(x + 1),
                            cell2posY(y) - wall_unit_px, pen);
    case MazeLib::Direction::North:
      return scene->addLine(cell2posX(x), cell2posY(y + 1),
                            cell2posX(x) + wall_unit_px, cell2posY(y + 1), pen);
    case MazeLib::Direction::West:
      return scene->addLine(cell2posX(x), cell2posY(y), cell2posX(x),
                            cell2posY(y) - wall_unit_px, pen);
    case MazeLib::Direction::South:
      return scene->addLine(cell2posX(x), cell2posY(y),
                            cell2posX(x) + wall_unit_px, cell2posY(y), pen);
    default:
      break;
    }
    return nullptr;
  }
  int cell2posX(int x) const {
    int w = wall_unit_px;
    return x * w;
  }
  int cell2posY(int y) const {
    int w = wall_unit_px;
    int s = MazeLib::MAZE_SIZE;
    return (s - y) * w;
  }
};

#endif // MAZESIMULATOR_H
