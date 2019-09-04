#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFileDialog>
#include <QMessageBox>
#include "Maze.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    scene(new QGraphicsScene),
    maze_simulator(ui, scene)
{
    ui->setupUi(this);
    ui->fileSeectEdit->setText("../mazedata/32MM2016HX.maze");
    ui->statusBar->showMessage("Hello World!");
    ui->mazeView->setScene(scene);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_fileSelectButton_clicked()
{
    QFileInfo fileinfo = QFileDialog::getOpenFileName(nullptr, "Select a Maze File", "../mazedata");
    /* get Relative Path */
    QDir pwd(".");
    QString s = pwd.relativeFilePath(fileinfo.filePath());
    ui->fileSeectEdit->setText(s);
    /* draw */
    on_drawButton_clicked();
}

void MainWindow::on_fileSeectEdit_returnPressed()
{
    on_drawButton_clicked();
}

void MainWindow::on_drawButton_clicked()
{
    QString filepath = ui->fileSeectEdit->text();
    /* Parse Maze File */
    MazeLib::Maze maze;
    if(!maze.parse(filepath.toStdString().c_str())){
        QMessageBox box(QMessageBox::Critical, "Parse Error", "Failed to Parse the Maze File!");
        box.exec();
        return;
    }
    /* Print Maze */
    maze_simulator.clear();
    maze_simulator.drawMaze(maze);
    /* Set the Maze */
    maze_simulator.setMazeTarget(maze);
    maze_simulator.replaceGoals(maze.getGoals());
}

void MainWindow::on_exitButton_clicked()
{
    exit(0);
}

void MainWindow::on_actionExit_triggered()
{
    exit(0);
}

void MainWindow::on_actionDraw_triggered()
{
    on_drawButton_clicked();
}

void MainWindow::on_resetButton_clicked()
{
    maze_simulator.reset();
}

void MainWindow::on_stepToggleButton_clicked()
{
    maze_simulator.toggle(ui->stepTimeBox->text().toInt());
}

void MainWindow::on_stepButton_clicked()
{
    maze_simulator.next(ui->stepCountBox->text().toInt());
}

void MainWindow::on_searchButton_clicked()
{
    /* Print Maze */
    maze_simulator.clear();
    maze_simulator.searchRun();
    maze_simulator.draw();
    /* Draw Shortest Path */
    if(!maze_simulator.drawShortest(maze_simulator.getMaze(), true)){
        QMessageBox box(QMessageBox::Warning, "Path Error", "Failed to Find any Shortest Path!");
        box.exec();
        return;
    }
}

void MainWindow::on_shortestDiagButton_clicked()
{
    /* Draw Shortest Path */
    if(!maze_simulator.drawShortest(maze_simulator.getMazeTarget(), true)){
        QMessageBox box(QMessageBox::Warning, "Path Error", "Failed to Find any Shortest Path!");
        box.exec();
        return;
    }
}

void MainWindow::on_shortestNoDiagButton_clicked()
{
    /* Draw Shortest Path */
    if(!maze_simulator.drawShortest(maze_simulator.getMazeTarget(), false)){
        QMessageBox box(QMessageBox::Warning, "Path Error", "Failed to Find any Shortest Path!");
        box.exec();
        return;
    }
}
