#-------------------------------------------------
#
# Project created by QtCreator 2019-08-12T14:12:55
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = mazelib_gui
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

INCLUDEPATH += \
    ../include

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    mazesimulator.cpp \
    ../src/Agent.cpp \
    ../src/Maze.cpp \
    ../src/RobotBase.cpp \
    ../src/SearchAlgorithm.cpp \
    ../src/ShortestAlgorithm.cpp \
    ../src/StepMap.cpp \
    ../src/StepMapSlalom.cpp \
    ../src/StepMapWall.cpp

HEADERS += \
    mainwindow.h \
    mazesimulator.h \
    ../include/Agent.h \
    ../include/CLRobotBase.h \
    ../include/Maze.h \
    ../include/RobotBase.h \
    ../include/SearchAlgorithm.h \
    ../include/ShortestAlgorithm.h \
    ../include/StepMap.h \
    ../include/StepMapSlalom.h \
    ../include/StepMapWall.h

FORMS += \
    mainwindow.ui

DISTFILES +=
