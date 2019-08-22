#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "mazesimulator.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_fileSelectButton_clicked();

    void on_drawButton_clicked();

    void on_exitButton_clicked();

    void on_actionExit_triggered();

    void on_actionDraw_triggered();

    void on_fileSeectEdit_returnPressed();

    void on_shortestDiagButton_clicked();

    void on_shortestNoDiagButton_clicked();

    void on_stepButton_clicked();

    void on_stepToggleButton_clicked();

    void on_searchButton_clicked();

    void on_resetButton_clicked();

private:
    Ui::MainWindow *ui;
    QGraphicsScene* scene;
    MazeSimulator maze_simulator;
};

#endif // MAINWINDOW_H
