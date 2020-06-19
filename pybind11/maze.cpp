/**
 * @file ctrl.cpp
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief this files defines a python module implemented in C++
 * @date 2020-06-11
 * @copyright Copyright (c) 2020 Ryotaro Onuki
 */
#include <Maze.h>

#include <pybind11/iostream.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <sstream>

PYBIND11_MODULE(MazeLib, m) {
  namespace py = pybind11;
  using namespace MazeLib;

  m.attr("MAZE_SIZE") = MAZE_SIZE;

  /* Direction */
  py::class_<Direction> direction(m, "Direction");
  direction.def(py::init<int>())
      .def(py::init<enum Direction::AbsoluteDirection>())
      .def("getAlong4", &Direction::getAlong4)
      .def("getDiag4", &Direction::getDiag4)
      .def("__str__", &Direction::toChar)
      .def("__int__", [](const Direction &d) { return uint8_t(d); })
      //
      ;
  py::enum_<Direction::AbsoluteDirection>(direction, "AbsoluteDirection")
      .value("East", Direction::East)
      .value("NorthEast", Direction::NorthEast)
      .value("North", Direction::North)
      .value("NorthWest", Direction::NorthWest)
      .value("West", Direction::West)
      .value("SouthWest", Direction::SouthWest)
      .value("South", Direction::South)
      .value("SouthEast", Direction::SouthEast)
      .export_values();
  py::enum_<Direction::RelativeDirection>(direction, "RelativeDirection")
      .value("Front", Direction::Front)
      .value("Left45", Direction::Left45)
      .value("Left", Direction::Left)
      .value("Left135", Direction::Left135)
      .value("Back", Direction::Back)
      .value("Right135", Direction::Right135)
      .value("Right", Direction::Right)
      .value("Right45", Direction::Right45)
      .export_values();
  py::implicitly_convertible<Direction::AbsoluteDirection, Direction>();
  py::class_<Directions>(m, "Directions");

  /* Position */
  py::class_<Position> position(m, "Position");
  position.attr("SIZE") = py::cast(Position::SIZE);
  position.def(py::init<int8_t, int8_t>(), py::arg("x") = 0, py::arg("y") = 0)
      .def_readwrite("x", &Position::x)
      .def_readwrite("y", &Position::y)
      .def("getIndex", &Position::getIndex)
      .def(py::self + py::self)
      .def(py::self - py::self)
      .def(py::self == py::self)
      .def(py::self != py::self)
      .def("next", &Position::next)
      .def("isInsideOfField", &Position::isInsideOfField)
      .def("rotate",
           py::overload_cast<const Direction, const Position &>(
               &Position::rotate, py::const_),
           py::arg("d"), py::arg("center") = Position())
      .def("__str__",
           [](const Position &obj) {
             std::stringstream ss;
             ss << obj;
             return ss.str();
           })
      //
      ;
  py::class_<Positions>(m, "Positions");

  /* Pose */
  py::class_<Pose>(m, "Pose")
      .def(py::init<Position &, Direction>(), py::arg("p") = Position(),
           py::arg("d") = Direction())
      .def_readwrite("p", &Pose::p)
      .def_readwrite("d", &Pose::d)
      .def("next", &Pose::next)
      .def("__str__",
           [](const Pose &obj) {
             std::stringstream ss;
             ss << obj;
             return ss.str();
           })
      //
      ;

  /* WallIndex */
  py::class_<WallIndex>(m, "WallIndex")
      .def(py::init<int8_t, int8_t, int8_t>(), py::arg("x") = 0,
           py::arg("y") = 0, py::arg("z") = 0)
      .def(py::init<const Position &, const Direction>())
      .def(py::init<const uint16_t>())
      .def(py::self == py::self)
      .def(py::self != py::self)
      .def("getIndex", &WallIndex::getIndex)
      .def("getPosition", &WallIndex::getPosition)
      .def("getDirection", &WallIndex::getDirection)
      .def("__str__",
           [](const WallIndex &obj) {
             std::stringstream ss;
             ss << obj;
             return ss.str();
           })
      .def("isInsideOfField", &WallIndex::isInsideOfField)
      .def("next", &WallIndex::next)
      .def("getNextDirection6", &WallIndex::getNextDirection6)
      //
      ;
  py::class_<WallIndexes>(m, "WallIndexes");

  /* WallRecord */
  py::class_<WallRecord>(m, "WallRecord")
      .def(py::init<>())
      .def(py::init<int8_t, int8_t, Direction, bool>(), py::arg("x") = 0,
           py::arg("y") = 0, py::arg("d") = Direction::Max,
           py::arg("b") = false)
      .def(py::init<const Position &, const Direction, bool>())
      .def("getPosition", &WallRecord::getPosition)
      .def("getDirection", &WallRecord::getDirection)
      //
      ;
  py::class_<WallRecords>(m, "WallRecords");

  /* Maze */
  py::class_<Maze>(m, "Maze")
      .def(py::init<>())
      .def(py::init<const Positions &, const Position &>(), //
           py::arg("goals") = Positions(), py::arg("start") = Position(0, 0))
      .def("reset", &Maze::reset, py::arg("set_start_wall") = true,
           py::arg("set_range_full") = false)
      /* isWall */
      .def("isWall",
           py::overload_cast<const WallIndex &>(&Maze::isWall, py::const_))
      .def("isWall", py::overload_cast<const Position &, const Direction>(
                         &Maze::isWall, py::const_))
      .def("isWall",
           py::overload_cast<const int8_t, const int8_t, const Direction>(
               &Maze::isWall, py::const_))
      /* setWall */
      .def("setWall",
           py::overload_cast<const WallIndex &, bool>(&Maze::setWall))
      .def("setWall",
           py::overload_cast<const Position &, const Direction, bool>(
               &Maze::setWall))
      .def("setWall",
           py::overload_cast<const int8_t, const int8_t, const Direction, bool>(
               &Maze::setWall))
      /* isKnown */
      .def("isKnown",
           py::overload_cast<const WallIndex &>(&Maze::isKnown, py::const_))
      .def("isKnown", py::overload_cast<const Position &, const Direction>(
                          &Maze::isKnown, py::const_))
      .def("isKnown",
           py::overload_cast<const int8_t, const int8_t, const Direction>(
               &Maze::isKnown, py::const_))
      /* setKnown */
      .def("setKnown",
           py::overload_cast<const WallIndex &, bool>(&Maze::setKnown))
      .def("setKnown",
           py::overload_cast<const Position &, const Direction, bool>(
               &Maze::setKnown))
      .def("setKnown",
           py::overload_cast<const int8_t, const int8_t, const Direction, bool>(
               &Maze::setKnown))
      /* canGo */
      .def("canGo",
           py::overload_cast<const WallIndex &>(&Maze::canGo, py::const_))
      .def("canGo", py::overload_cast<const Position &, const Direction>(
                        &Maze::canGo, py::const_))
      /* others */
      .def("updateWall",
           py::overload_cast<const Position &, const Direction, bool, bool>(
               &Maze::updateWall),
           py::arg("p"), py::arg("d"), py::arg("b"), py::arg("pushLog") = true)
      .def("resetLastWalls", &Maze::resetLastWalls)
      .def("wallCount", &Maze::wallCount)
      .def("unknownCount", &Maze::unknownCount)
      /* print */
      .def("print",
           [](const Maze &maze) {
             py::scoped_ostream_redirect stream(
                 std::cout, py::module::import("sys").attr("stdout"));
             maze.print();
           })
      .def("print",
           [](const Maze &maze, const int maze_size) {
             py::scoped_ostream_redirect stream(
                 std::cout, py::module::import("sys").attr("stdout"));
             maze.print(std::cout, maze_size);
           })
      .def("print",
           [](const Maze &maze, const Directions &dirs, const Position &start,
              const int maze_size) {
             py::scoped_ostream_redirect stream(
                 std::cout, py::module::import("sys").attr("stdout"));
             maze.print(dirs, start, std::cout, maze_size);
           })
      /* parse */
      .def("parse", py::overload_cast<const std::string &>(&Maze::parse))
      /* setters and getters */
      .def("setGoals", &Maze::setGoals)
      .def("setStart", &Maze::setStart)
      .def("getGoals", &Maze::getGoals)
      .def("getStart", &Maze::getStart)
      .def("getWallRecords", &Maze::getWallRecords)
      .def("getMinX", &Maze::getMinX)
      .def("getMinY", &Maze::getMinY)
      .def("getMaxX", &Maze::getMaxX)
      .def("getMaxY", &Maze::getMaxY)
      //
      ;
}
