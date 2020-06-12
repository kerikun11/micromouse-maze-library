/**
 * @file ctrl.cpp
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief this files defines a python module implemented in C++
 * @date 2020-06-11
 * @copyright Copyright (c) 2020 Ryotaro Onuki
 */
#include <Maze.h>

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
      .def("getAlong4", &Direction::getAlong4)
      .def("getDiag4", &Direction::getDiag4)
      .def("__str__", &Direction::toChar)
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
}
