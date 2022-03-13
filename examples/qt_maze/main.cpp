#include <MazeLib/Maze.h>

#include <QtWidgets/QApplication>
#include <QtWidgets/QGraphicsLineItem>
#include <QtWidgets/QGraphicsView>

class QMazeView : public QGraphicsView {
 public:
  QMazeView(QGraphicsScene* scene = new QGraphicsScene()) : scene(scene) {
    // this->setTransform(QTransform(1, 0, 0, -1, 0, 0));
    this->setScene(scene);
    const int window_size = MazeLib::MAZE_SIZE * px_wall + 64;
    this->resize(window_size, window_size);
    scene->setBackgroundBrush(Qt::black);
    for (int z = 0; z < 2; ++z)
      for (int x = -1; x < MazeLib::MAZE_SIZE; ++x)
        for (int y = -1; y < MazeLib::MAZE_SIZE; ++y)
          if ((z == 0 && y >= 0) || (z == 1 && x >= 0)) {
            const auto wi = MazeLib::WallIndex(x, y, z);
            const auto wl =
                scene->addLine(getWallLine(wi), getWallPen(true, true));
            if (wi.isInsideOfField())
              wall_lines[wi.getIndex()] = wl;
          }
    for (int i = 0; i < MazeLib::MAZE_SIZE; ++i) {
      const int w = px_wall;
      const int s = MazeLib::MAZE_SIZE;
      scene->addText(QString::number(i))->setPos((i + 0.25) * w, s * w);
      scene->addText(QString::number(i))->setPos(-w * 0.9, (s - i - 1) * w);
    }
  }
  void drawMaze(const MazeLib::Maze& maze) {
    for (int z = 0; z < 2; ++z)
      for (int x = 0; x < MazeLib::MAZE_SIZE; ++x)
        for (int y = 0; y < MazeLib::MAZE_SIZE; ++y)
          drawWall(maze, MazeLib::WallIndex(x, y, z));
  }
  void drawWall(const MazeLib::Maze& maze, const MazeLib::WallIndex& wi) {
    if (!wi.isInsideOfField())
      return;
    const auto is_wall = maze.isWall(wi);
    const auto is_known = maze.isKnown(wi);
    wall_lines[wi.getIndex()]->setPen(getWallPen(is_wall, is_known));
  }
  const auto getScene() const { return scene; }

 protected:
  QGraphicsScene* scene;
  std::array<QGraphicsLineItem*, MazeLib::WallIndex::SIZE> wall_lines;
  int px_wall = 28;

  QLine getWallLine(MazeLib::WallIndex wi) const {
    const int s = MazeLib::MAZE_SIZE;
    const int x = wi.x;
    const int y = s - wi.y;
    const int w = px_wall;
    switch (wi.getDirection()) {
      case MazeLib::Direction::East:
        return QLine(w * x + w, w * y, w * x + w, w * y - w);
      case MazeLib::Direction::North:
        return QLine(w * x, w * y - w, w * x + w, w * y - w);
    }
    assert(false);
    return QLine();
  }
  QPen getWallPen(bool is_wall, bool is_known) const {
    QPen pen(Qt::NoPen);
    if (!is_known) {
      pen.setColor(Qt::red);
      pen.setStyle(Qt::PenStyle::DotLine);
      return pen;
    }
    if (is_wall) {
      pen.setColor(Qt::red);
      pen.setStyle(Qt::PenStyle::SolidLine);
      return pen;
    }
    pen.setColor(QColor(64, 64, 64));
    pen.setStyle(Qt::PenStyle::DotLine);
    return pen;
  }
  QGraphicsLineItem* getWallLineItem(MazeLib::WallIndex wi) const {
    return wi.isInsideOfField() ? wall_lines[wi.getIndex()]
                                : new QGraphicsLineItem(getWallLine(wi));
  }
};

int main(int argc, char* argv[]) {
  QApplication a(argc, argv);

  const auto filepath = "../mazedata/data/32MM2019HX.maze";
  MazeLib::Maze maze;
  if (!maze.parse(filepath))
    return std::cout << "Failed to Parse File!" << std::endl, 1;

  QMazeView* maze_view = new QMazeView();
  maze_view->drawMaze(maze);
  maze_view->show();

  return a.exec();
}
