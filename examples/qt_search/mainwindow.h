#pragma once

#include <MazeLib/Maze.h>
#include <MazeLib/StepMapSlalom.h>

#include <QFileDialog>
#include <QMainWindow>
#include <QMessageBox>

#include "mazesimulator.h"
#include "ui_mainwindow.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  MainWindow(QWidget* parent = 0)
      : QMainWindow(parent),
        ui(new Ui::MainWindow),
        scene(new QGraphicsScene),
        mazeSimulator(ui, scene) {
    ui->setupUi(this);
    ui->fileSelectEdit->setText("../mazedata/data/32MM2016HX.maze");
    ui->statusBar->showMessage("Hello World!");
    ui->mazeView->setScene(scene);
  }

  ~MainWindow() { delete ui; }

 private slots:

  void on_fileSelectButton_clicked() {
    QFileInfo fileinfo = QFileDialog::getOpenFileName(
        nullptr, "Select a Maze File", "../mazedata/data");
    /* get Relative Path */
    QDir pwd(".");
    QString s = pwd.relativeFilePath(fileinfo.filePath());
    ui->fileSelectEdit->setText(s);
    /* draw */
    on_drawButton_clicked();
  }

  void on_drawButton_clicked() {
    QString filepath = ui->fileSelectEdit->text();
    /* Parse Maze File */
    MazeLib::Maze maze;
    if (!maze.parse(filepath.toStdString().c_str())) {
      QMessageBox box(QMessageBox::Critical, "Parse Error",
                      "Failed to Parse the Maze File!");
      box.exec();
      return;
    }
    /* Print Maze */
    mazeSimulator.clear();
    mazeSimulator.drawMaze(maze);
    /* Set the Maze */
    mazeSimulator.setMazeTarget(maze);
    mazeSimulator.replaceGoals(maze.getGoals());
  }

  void on_drawAllButton_clicked() {
    on_drawButton_clicked();
    on_stepmapSimpleButton_clicked();
    on_stepmapWallSimpleButton_clicked();
    // on_stepmapTrapezoidButton_clicked();
    // on_stepmapWallTrapezoidButton_clicked();
    on_shortestNoDiagButton_clicked();
    on_shortestDiagButton_clicked();
  }

  void on_shortestDiagButton_clicked() {
    /* Draw Shortest Path */
    {
      // for (const auto &factor : {0.5f, 1.0f, 2.0f, 4.0f}) {
      //    StepMapSlalom::EdgeCost::RunParameter rp;
      //    rp.am_a *= factor;
      //    rp.am_d *= factor;
      //    rp.vm_a *= factor * factor;
      //    rp.vm_d *= factor * factor;
      //    StepMapSlalom::EdgeCost edgeCost(rp);
      const auto& maze = mazeSimulator.getMazeTarget();
      if (!mazeSimulator.drawShortest(maze, true, edgeCost)) {
        QMessageBox box(QMessageBox::Warning, "Path Error",
                        "Failed to Find any Shortest Path!");
        box.exec();
        return;
      }
    }
  }

  void on_shortestNoDiagButton_clicked() {
    /* Draw Shortest Path */
    {
      // for (const auto factor : {0.5f, 1.0f, 2.0f, 4.0f}) {
      //    StepMapSlalom::EdgeCost::RunParameter rp;
      //    rp.am_a *= factor;
      //    rp.am_d *= factor;
      //    rp.vm_a *= factor * factor;
      //    rp.vm_d *= factor * factor;
      //    StepMapSlalom::EdgeCost edgeCost(rp);
      const auto& maze = mazeSimulator.getMazeTarget();
      if (!mazeSimulator.drawShortest(maze, false, edgeCost)) {
        QMessageBox box(QMessageBox::Warning, "Path Error",
                        "Failed to Find any Shortest Path!");
        box.exec();
        return;
      }
    }
  }

  void on_stepmapSimpleButton_clicked() {
    const auto& maze = mazeSimulator.getMazeTarget();
    if (!mazeSimulator.drawShortestStepMap(maze, true)) {
      QMessageBox box(QMessageBox::Warning, "Path Error",
                      "Failed to Find any Shortest Path!");
      box.exec();
      return;
    }
  }

  void on_stepmapTrapezoidButton_clicked() {
    const auto& maze = mazeSimulator.getMazeTarget();
    if (!mazeSimulator.drawShortestStepMap(maze, false)) {
      QMessageBox box(QMessageBox::Warning, "Path Error",
                      "Failed to Find any Shortest Path!");
      box.exec();
      return;
    }
  }

  void on_stepmapWallSimpleButton_clicked() {
    const auto& maze = mazeSimulator.getMazeTarget();
    if (!mazeSimulator.drawShortestStepMapWall(maze, true)) {
      QMessageBox box(QMessageBox::Warning, "Path Error",
                      "Failed to Find any Shortest Path!");
      box.exec();
      return;
    }
  }

  void on_stepmapWallTrapezoidButton_clicked() {
    const auto& maze = mazeSimulator.getMazeTarget();
    if (!mazeSimulator.drawShortestStepMapWall(maze, false)) {
      QMessageBox box(QMessageBox::Warning, "Path Error",
                      "Failed to Find any Shortest Path!");
      box.exec();
      return;
    }
  }

  void on_fileSelectEdit_returnPressed() {
    on_drawButton_clicked();
    /* StepMap StepMapWall */
    on_stepmapSimpleButton_clicked();
    on_stepmapWallSimpleButton_clicked();
    /* StepMapSlalom */
    on_shortestNoDiagButton_clicked();
    on_shortestDiagButton_clicked();
    /* save */
    on_saveImageButton_clicked();
  }

  void on_exitButton_clicked() { exit(0); }

  void on_actionExit_triggered() { exit(0); }

  void on_actionDraw_triggered() { on_drawButton_clicked(); }

  void on_resetButton_clicked() { mazeSimulator.reset(); }

  void on_stepToggleButton_clicked() {
    //  mazeSimulator.toggle(ui->stepTimeBox->text().toInt());
    mazeSimulator.toggle(1);
  }

  void on_stepButton_clicked() {
    //  mazeSimulator.next(ui->stepCountBox->text().toInt());
    mazeSimulator.next(1);
  }

  void on_searchButton_clicked() {
    /* Print Maze */
    mazeSimulator.clear();
    mazeSimulator.searchRun();
    mazeSimulator.clear();
    mazeSimulator.drawMaze(mazeSimulator.getMaze());
    /* Draw Shortest Path */
    on_shortestDiagButton_clicked();
    on_shortestNoDiagButton_clicked();
  }

  void on_saveImageButton_clicked() {
    auto filepath = QFileInfo(ui->fileSelectEdit->text());
    scene->clearSelection();  // Selections would also render to the file
    scene->setSceneRect(scene->itemsBoundingRect());  // Re-shrink the scene to
                                                      // it's bounding contents
    QImage image(scene->sceneRect().size().toSize(),
                 QImage::Format_ARGB32);  // Create the image with the exact
                                          // size of the shrunk scene
    image.fill(Qt::transparent);          // Start all pixels transparent

    QPainter painter(&image);
    scene->render(&painter);
    image.save(filepath.fileName() + ".png");
  }

  void on_slalomCostF45Box_valueChanged(int arg1) {
    on_slalomCostBox_valueChanged(StepMapSlalom::Slalom::F45, arg1);
  }
  void on_slalomCostF90Box_valueChanged(int arg1) {
    on_slalomCostBox_valueChanged(StepMapSlalom::Slalom::F90, arg1);
  }
  void on_slalomCostF135Box_valueChanged(int arg1) {
    on_slalomCostBox_valueChanged(StepMapSlalom::Slalom::F135, arg1);
  }
  void on_slalomCostF180Box_valueChanged(int arg1) {
    on_slalomCostBox_valueChanged(StepMapSlalom::Slalom::F180, arg1);
  }
  void on_slalomCostFV90Box_valueChanged(int arg1) {
    on_slalomCostBox_valueChanged(StepMapSlalom::Slalom::FV90, arg1);
  }
  void on_slalomCostFS90Box_valueChanged(int arg1) {
    on_slalomCostBox_valueChanged(StepMapSlalom::Slalom::FS90, arg1);
  }

  void on_straightCostVmAlong_valueChanged(int value) {
    auto rp = edgeCost.getRunParameter();
    StepMapSlalom::EdgeCost::RunParameter rp_default;
    if (value > 0)
      rp.vm_a = value * StepMapSlalom::EdgeCost::RunParameter::factor;
    else
      rp.vm_a = rp_default.vm_a;
    edgeCost.setRunParameter(rp);
    /* draw */
    on_drawButton_clicked();
    on_shortestDiagButton_clicked();
    on_shortestNoDiagButton_clicked();
  }
  void on_straightCostVmDiag_valueChanged(int value) {
    auto rp = edgeCost.getRunParameter();
    StepMapSlalom::EdgeCost::RunParameter rp_default;
    if (value > 0)
      rp.vm_d = value * StepMapSlalom::EdgeCost::RunParameter::factor;
    else
      rp.vm_d = rp_default.vm_d;
    edgeCost.setRunParameter(rp);
    /* draw */
    on_drawButton_clicked();
    on_shortestDiagButton_clicked();
    on_shortestNoDiagButton_clicked();
  }
  void on_straightCostAmAlong_valueChanged(int value) {
    auto rp = edgeCost.getRunParameter();
    StepMapSlalom::EdgeCost::RunParameter rp_default;
    if (value > 0)
      rp.am_a = value * StepMapSlalom::EdgeCost::RunParameter::factor *
                StepMapSlalom::EdgeCost::RunParameter::factor;
    else
      rp.am_a = rp_default.am_a;
    edgeCost.setRunParameter(rp);
    /* draw */
    on_drawButton_clicked();
    on_shortestDiagButton_clicked();
    on_shortestNoDiagButton_clicked();
  }
  void on_straightCostAmDiag_valueChanged(int value) {
    auto rp = edgeCost.getRunParameter();
    StepMapSlalom::EdgeCost::RunParameter rp_default;
    if (value > 0)
      rp.am_d = value * StepMapSlalom::EdgeCost::RunParameter::factor *
                StepMapSlalom::EdgeCost::RunParameter::factor;
    else
      rp.am_d = rp_default.am_d;
    edgeCost.setRunParameter(rp);
    /* draw */
    on_drawButton_clicked();
    on_shortestDiagButton_clicked();
    on_shortestNoDiagButton_clicked();
  }

 private:
  Ui::MainWindow* ui;
  QGraphicsScene* scene;
  MazeSimulator mazeSimulator;
  StepMapSlalom::EdgeCost edgeCost;

  void on_slalomCostBox_valueChanged(StepMapSlalom::Slalom slalom, int value) {
    auto rp = edgeCost.getRunParameter();
    StepMapSlalom::EdgeCost::RunParameter rp_default;
    if (value > 0)
      rp.slalomCostTable[slalom] =
          value / StepMapSlalom::EdgeCost::RunParameter::factor;
    else
      rp.slalomCostTable[slalom] = rp_default.slalomCostTable[slalom];
    edgeCost.setRunParameter(rp);
    /* draw */
    on_drawButton_clicked();
    on_shortestDiagButton_clicked();
    on_shortestNoDiagButton_clicked();
  }
};
