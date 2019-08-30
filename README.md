# MazeLib

## build

### command line test

```sh
mkdir build
cd build
cmake ..
make main
```

### cross compiling

```sh
mkdir build
cd build
cmake -DCROSS_COMPILING=1 -DCMAKE_TOOLCHAIN_FILE=../cmake/toolchain_xtensa-esp32-elf.cmake ..
make
```

## Class Design

- Dir
- Vector
- WallLog
- Maze

- StepMap
- SearchAlgorithm
- Agent
- RobotBase
- CLRobotBase
