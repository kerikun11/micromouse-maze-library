#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "Maze.h"
#include "StepMapSlalom.h"
#include <QFileDialog>
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), scene(new QGraphicsScene),
      maze_simulator(ui, scene) {
  ui->setupUi(this);
  ui->fileSeectEdit->setText("../mazedata/32MM2016HX.maze");
  ui->statusBar->showMessage("Hello World!");
  ui->mazeView->setScene(scene);
}

MainWindow::~MainWindow() { delete ui; }

void MainWindow::on_fileSelectButton_clicked() {
  QFileInfo fileinfo = QFileDialog::getOpenFileName(
      nullptr, "Select a Maze File", "../mazedata");
  /* get Relative Path */
  QDir pwd(".");
  QString s = pwd.relativeFilePath(fileinfo.filePath());
  ui->fileSeectEdit->setText(s);
  /* draw */
  on_drawButton_clicked();
}

void MainWindow::on_drawButton_clicked() {
  QString filepath = ui->fileSeectEdit->text();
  /* Parse Maze File */
  MazeLib::Maze maze;
  if (!maze.parse(filepath.toStdString().c_str())) {
    QMessageBox box(QMessageBox::Critical, "Parse Error",
                    "Failed to Parse the Maze File!");
    box.exec();
    return;
  }
  /* Print Maze */
  maze_simulator.clear();
  maze_simulator.drawMaze(maze);
  maze_simulator.maze = maze;
}

void MainWindow::on_drawAllButton_clicked() {
  on_drawButton_clicked();
  on_shortestDiagButton_clicked();
  on_shortestNoDiagButton_clicked();
  on_stepmapSimpleButton_clicked();
  on_stepmapTrapezoidButton_clicked();
  on_stepmapWallSimpleButton_clicked();
  on_stepmapWallTrapezoidButton_clicked();
}

void MainWindow::on_shortestDiagButton_clicked() {
  /* Draw Shortest Path */
  StepMapSlalom::EdgeCost::RunParameter rp;
  StepMapSlalom::EdgeCost edge_cost(rp);
  if (!maze_simulator.drawShortest(maze_simulator.maze, true)) {
    QMessageBox box(QMessageBox::Warning, "Path Error",
                    "Failed to Find any Shortest Path!");
    box.exec();
    return;
  }
}

void MainWindow::on_shortestNoDiagButton_clicked() {
  /* Draw Shortest Path */
  if (!maze_simulator.drawShortest(maze_simulator.maze, false)) {
    QMessageBox box(QMessageBox::Warning, "Path Error",
                    "Failed to Find any Shortest Path!");
    box.exec();
    return;
  }
}

void MainWindow::on_stepmapSimpleButton_clicked() {
  if (!maze_simulator.drawShortestStepMap(maze_simulator.maze, true)) {
    QMessageBox box(QMessageBox::Warning, "Path Error",
                    "Failed to Find any Shortest Path!");
    box.exec();
    return;
  }
}

void MainWindow::on_stepmapTrapezoidButton_clicked() {
  if (!maze_simulator.drawShortestStepMap(maze_simulator.maze, false)) {
    QMessageBox box(QMessageBox::Warning, "Path Error",
                    "Failed to Find any Shortest Path!");
    box.exec();
    return;
  }
}

void MainWindow::on_stepmapWallSimpleButton_clicked() {
  if (!maze_simulator.drawShortestStepMapWall(maze_simulator.maze, true)) {
    QMessageBox box(QMessageBox::Warning, "Path Error",
                    "Failed to Find any Shortest Path!");
    box.exec();
    return;
  }
}

void MainWindow::on_stepmapWallTrapezoidButton_clicked() {
  if (!maze_simulator.drawShortestStepMapWall(maze_simulator.maze, false)) {
    QMessageBox box(QMessageBox::Warning, "Path Error",
                    "Failed to Find any Shortest Path!");
    box.exec();
    return;
  }
}

void MainWindow::on_fileSeectEdit_returnPressed() {
  on_drawButton_clicked();
  /* add! */
  on_shortestDiagButton_clicked();
  on_shortestNoDiagButton_clicked();
  on_saveImageButton_clicked();
}

void MainWindow::on_exitButton_clicked() { exit(0); }

void MainWindow::on_actionExit_triggered() { exit(0); }

void MainWindow::on_actionDraw_triggered() { on_drawButton_clicked(); }

void MainWindow::on_saveImageButton_clicked() {
  auto filepath = QFileInfo(ui->fileSeectEdit->text());
  scene->clearSelection(); // Selections would also render to the file
  scene->setSceneRect(scene->itemsBoundingRect()); // Re-shrink the scene to
                                                   // it's bounding contents
  QImage image(scene->sceneRect().size().toSize(),
               QImage::Format_ARGB32); // Create the image with the exact size
                                       // of the shrunk scene
  image.fill(Qt::transparent);         // Start all pixels transparent

  QPainter painter(&image);
  scene->render(&painter);
  image.save(filepath.fileName() + ".png");
}
