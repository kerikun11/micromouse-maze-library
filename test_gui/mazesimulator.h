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

  void clear() {
    /* set Background Color */
    scene->clear();
    scene->setBackgroundBrush(Qt::black);
    /* Print Line Number */
    int w = wall_unit_px;
    int s = MazeLib::MAZE_SIZE;
    for (int i = 0; i < MazeLib::MAZE_SIZE; ++i) {
      scene->addText(QString::number(i))->setPos((i + 0.25) * w, s * w);
      scene->addText(QString::number(i))->setPos(-w, (s - i - 1) * w);
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
  void drawMaze(const MazeLib::Maze &maze) {
    /* Print Walls */
    for (int x = 0; x < MazeLib::MAZE_SIZE + 1; ++x)
      for (int y = 0; y < MazeLib::MAZE_SIZE + 1; ++y) {
        for (const auto d : {MazeLib::Direction::West, MazeLib::Direction::South}) {
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
  }
  void drawStep(const StepMap &map) {
    for (int x = 0; x < MazeLib::MAZE_SIZE; ++x)
      for (int y = 0; y < MazeLib::MAZE_SIZE; ++y) {
        QFont font;
        font.setPointSize(5);
        scene
            ->addText(QString::number(std::min((int)map.getStep(x, y), 99999)),
                      font)
            ->setPos(cell2posX(x), cell2posY(y + 1));
        /* Draw Wall */
        QPen pen(Qt::red);
      }
  }
  void drawPose(const Pose &pose) {
    /* Draw Machine */
    const auto p = pose.first;
    const auto d = pose.second;
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
  bool drawShortest(const Maze &maze, const bool diag_enabled) {
    Maze maze_tmp = maze;
    SearchAlgorithm sa(maze_tmp);
    Directions dirs;
    if (!sa.calcShortestDirections(dirs, diag_enabled))
      return false;
    auto p = maze.getStart();
    for (size_t i = 0; i < dirs.size(); ++i) {
      const auto d = dirs[i];
      const auto next_p = p.next(d);
      QPoint p1, p2;
      if (diag_enabled) {
        if (i == dirs.size() - 1)
          continue;
        const auto nd = dirs[i + 1];
        p1 = p2 = QPoint(wall_unit_px / 2, 0);
        QMatrix m1, m2;
        m1.rotate(-45 * d);
        m2.rotate(-45 * nd);
        p1 = m1.map(p1);
        p2 = m2.map(p2);
      }
      QPen pen(Qt::yellow);
      pen.setWidth(2);
      scene->addLine(p1.x() + cell2posX(p.x) + wall_unit_px / 2,
                     p1.y() + cell2posY(p.y) - wall_unit_px / 2,
                     p2.x() + cell2posX(next_p.x) + wall_unit_px / 2,
                     p2.y() + cell2posY(next_p.y) - wall_unit_px / 2, pen);
      p = next_p;
    }
    return true;
  }
  Maze &getMazeTarget() { return maze_target; }
  void setMazeTarget(const Maze &maze) { maze_target = maze; }
  bool endFastRunBackingToStartRun() {
    /* エラー処理 */
    if (getShortestDirections().empty()) {
      logw << "ShortestDirections are empty!" << std::endl;
      return false;
    }
    /* real を最短後の位置に移す */
    Position p = maze.getStart();
    for (const auto d : getShortestDirections())
      p = p.next(d);
    real = Pose(p, getShortestDirections().back());
    /* 基底関数を呼ぶ */
    return RobotBase::endFastRunBackingToStartRun();
  }
  void drawStatus() {
    std::stringstream ss;
    ss << "State: " << SearchAlgorithm::stateString(getState());
    ss << "\t";
    ss << "Pos: " << Pose{getCurrentPosition(), getCurrentDirection()};
    ui->statusBar->showMessage(ss.str().c_str());
  }
  void draw() {
    clear();
    drawMaze(maze);
    drawStep(getSearchAlgorithm().getStepMap());
    drawPose(real);
    drawStatus();
  }

protected:
  Maze maze;
  Maze maze_target;
  Pose fake_offset;
  Pose real = {Position(0, 0), Direction::North};

  virtual void findWall(bool &left, bool &front, bool &right,
                        bool &back) override {
    left = maze_target.isWall(real.first, real.second + Direction::Left);
    front = maze_target.isWall(real.first, real.second + Direction::Front);
    right = maze_target.isWall(real.first, real.second + Direction::Right);
    back = maze_target.isWall(real.first, real.second + Direction::Back);
#if 0
        /* 前1区画先の壁を読める場合 */
        if (!front)
            updateWall(current_position.next(current_direction), current_direction,
                       maze_target.isWall(real.first.next(real.second), real.second));
#endif
  }
  virtual void calcNextDirectionsPreCallback() override {
    t_start = std::chrono::system_clock::now();
  }
  virtual void calcNextDirectionsPostCallback(SearchAlgorithm::State prevState
                                        __attribute__((unused)),
                                        SearchAlgorithm::State newState
                                        __attribute__((unused))) override {
    end = std::chrono::system_clock::now();
    usec = std::chrono::duration_cast<std::chrono::microseconds>(end - t_start)
               .count();
    max_usec = std::max(max_usec, usec);
    if (newState == prevState)
      return;
    /* State Change has occurred */
    if (prevState == SearchAlgorithm::IDENTIFYING_POSITION) {
      min_id_wall = std::min(
          min_id_wall, getSearchAlgorithm().getIdMaze().getWallLogs().size());
      max_id_wall = std::max(
          max_id_wall, getSearchAlgorithm().getIdMaze().getWallLogs().size());
    }
    if (newState == SearchAlgorithm::IDENTIFYING_POSITION) {
    }
    if (newState == SearchAlgorithm::SEARCHING_ADDITIONALLY) {
    }
    if (newState == SearchAlgorithm::BACKING_TO_START) {
    }
    if (newState == SearchAlgorithm::REACHED_START) {
    }
  }
  virtual void discrepancyWithKnownWall() override {
    if (getState() != SearchAlgorithm::IDENTIFYING_POSITION) {
      printInfo();
      std::cout
          << "There was a discrepancy with known information! CurrentPose:\t"
          << Pose{getCurrentPosition(), getCurrentDirection()} << std::endl;
    }
  }
  virtual void crashed() {
    std::cerr << "The robot crashed into the wall! fake_offset:\t"
              << fake_offset << "\treal:\t" << real << std::endl;
  }
  virtual void queueAction(const Action action) override {
    /* draw */
    draw();
    /* block */
    int code = loop->exec();
    if (code < 0)
      return;
    /* release */
    cost += getTimeCost(action);
    step++;
    switch (action) {
    case RobotBase::START_STEP:
      real.first = Position(0, 1);
      real.second = Direction::North;
      f++;
      break;
    case RobotBase::START_INIT:
      real.second = real.second + Direction::Back;
      if (!maze_target.canGo(real.first, real.second))
        crashed();
      real.first = real.first.next(real.second);
      break;
    case RobotBase::ST_HALF_STOP:
      break;
    case RobotBase::TURN_L:
      real.second = real.second + Direction::Left;
      if (!maze_target.canGo(real.first, real.second))
        crashed();
      real.first = real.first.next(real.second);
      l++;
      break;
    case RobotBase::TURN_R:
      real.second = real.second + Direction::Right;
      if (!maze_target.canGo(real.first, real.second))
        crashed();
      real.first = real.first.next(real.second);
      r++;
      break;
    case RobotBase::ROTATE_180:
      real.second = real.second + Direction::Back;
      if (!maze_target.canGo(real.first, real.second))
        crashed();
      real.first = real.first.next(real.second);
      b++;
      break;
    case RobotBase::ST_FULL:
      if (!maze_target.canGo(real.first, real.second))
        crashed();
      real.first = real.first.next(real.second);
      f++;
      break;
    case RobotBase::ST_HALF:
      break;
    }
  }
  virtual float getTimeCost(const Action action) {
    const float velocity = 240.0f;
    const float segment = 90.0f;
    switch (action) {
    case RobotBase::START_STEP:
      return 1.0f;
    case RobotBase::START_INIT:
      return 3.0f;
    case RobotBase::ST_HALF_STOP:
      return segment / 2 / velocity;
    case RobotBase::TURN_L:
      return 71 / velocity;
    case RobotBase::TURN_R:
      return 71 / velocity;
    case RobotBase::ROTATE_180:
      return 2.0f;
    case RobotBase::ST_FULL:
      return segment / velocity;
    case RobotBase::ST_HALF:
      return segment / 2 / velocity;
    }
    return 0;
  }

private:
  int step = 0, f = 0, l = 0, r = 0, b = 0; /**< 探索の評価のためのカウンタ */
  float cost = 0;
  int max_usec = 0;
  int usec = 0;
  size_t max_id_wall = 0;
  size_t min_id_wall = MAZE_SIZE * MAZE_SIZE * 4;
  std::chrono::system_clock::time_point t_start;
  std::chrono::system_clock::time_point end;

private:
  QEventLoop *loop = new QEventLoop();
  QTimer *timer = new QTimer();
  Ui::MainWindow *ui;
  QGraphicsScene *scene;
  int wall_unit_px = 28;
  int pillar_px = 2;
  int wall_px = wall_unit_px - pillar_px;

  QGraphicsItem *addWall(QGraphicsScene *scene, MazeLib::Pose pose,
                         const QPen &pen) {
    int x = pose.first.x;
    int y = pose.first.y;
    MazeLib::Direction d = pose.second;
    switch (d) {
    case MazeLib::Direction::East:
      return scene->addLine(cell2posX(x + 1), cell2posY(y) - pillar_px / 2,
                            cell2posX(x + 1),
                            cell2posY(y) - pillar_px / 2 - wall_px, pen);
    case MazeLib::Direction::North:
      return scene->addLine(cell2posX(x) + pillar_px / 2, cell2posY(y + 1),
                            cell2posX(x) + pillar_px / 2 + wall_px,
                            cell2posY(y + 1), pen);
    case MazeLib::Direction::West:
      return scene->addLine(cell2posX(x), cell2posY(y) - pillar_px / 2,
                            cell2posX(x),
                            cell2posY(y) - pillar_px / 2 - wall_px, pen);
    case MazeLib::Direction::South:
      return scene->addLine(cell2posX(x) + pillar_px / 2, cell2posY(y),
                            cell2posX(x) + pillar_px / 2 + wall_px,
                            cell2posY(y), pen);
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
