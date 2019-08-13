#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFileDialog>
#include <QMessageBox>
#include "Maze.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    scene(new QGraphicsScene),
    maze_simulator(scene)
{
    ui->setupUi(this);
    ui->fileSeectEdit->setText("../mazedata/32MM2017HX.maze");
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
    QDir pwd(".");
    QString s = pwd.relativeFilePath(fileinfo.filePath());
    ui->fileSeectEdit->setText(s);
    on_fileSeectEdit_returnPressed();
}

void MainWindow::on_drawButton_clicked()
{
    QString filepath = ui->fileSeectEdit->text();
    /* Parse Maze File */
    MazeLib::Maze maze;
    if(!maze.parse(filepath.toStdString().c_str())){
        QMessageBox box;
        box.setWindowTitle("Parse Error");
        box.setText("Failed to Parse the Maze File!");
        box.setIcon(QMessageBox::Critical);
        box.exec();
        return;
    }
    /* Print Maze */
    maze_simulator.clear();
    maze_simulator.drawMaze(maze);
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

void MainWindow::on_startButton_clicked()
{
    QString filepath = ui->fileSeectEdit->text();
    /* Parse Maze File */
    MazeLib::Maze maze;
    if(!maze.parse(filepath.toStdString().c_str())){
        QMessageBox box;
        box.setWindowTitle("Parse Error");
        box.setText("Failed to Parse the Maze File!");
        box.setIcon(QMessageBox::Critical);
        box.exec();
        return;
    }
    /* Print Maze */
    maze_simulator.clear();
    maze_simulator.setMazeTarget(maze);
    maze_simulator.replaceGoals(maze.getGoals());
    maze_simulator.searchRun();
}

void MainWindow::on_fileSeectEdit_returnPressed()
{
    on_drawButton_clicked();
}
