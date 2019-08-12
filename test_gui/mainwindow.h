#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "mazedrawer.h"

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

private:
    Ui::MainWindow *ui;
    MazeDrawer maze_drawer;
};

#endif // MAINWINDOW_H
