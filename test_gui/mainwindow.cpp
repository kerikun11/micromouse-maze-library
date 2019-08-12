#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFileDialog>
#include "Maze.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->fileSeectEdit->setText("../mazedata/32MM2018HX.maze");
    ui->statusBar->showMessage("Hello World!");
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_fileSelectButton_clicked()
{
    QFileInfo fileinfo = QFileDialog::getOpenFileName(nullptr, "Select a Maze File");
    QDir pwd(".");
    QString s = pwd.relativeFilePath(fileinfo.filePath());
    ui->fileSeectEdit->setText(s);
}

void MainWindow::on_drawButton_clicked()
{
    QString filepath = ui->fileSeectEdit->text();
    MazeLib::Maze maze(filepath.toStdString().c_str());
    QGraphicsScene* scene = new QGraphicsScene();
    maze_drawer.clear(scene);
    maze_drawer.draw(scene, maze);
    ui->mazeView->setScene(scene);
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
