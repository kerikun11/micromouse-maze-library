/**
 * @file ctrl.cpp
 * @brief this files defines a python module implemented in C++
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2020-06-11
 * @copyright Copyright 2020 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#include <MazeLib/Maze.h>
#include <MazeLib/StepMap.h>
#include <MazeLib/StepMapSlalom.h>
#include <MazeLib/StepMapWall.h>

#include <pybind11/iostream.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include <sstream>

// see
// https://pybind11.readthedocs.io/en/stable/advanced/cast/stl.html#making-opaque-types
PYBIND11_MAKE_OPAQUE(MazeLib::Directions);
PYBIND11_MAKE_OPAQUE(MazeLib::Positions);

PYBIND11_MODULE(MazeLib, m) {
  namespace py = pybind11;
  using namespace MazeLib;

  m.attr("MAZE_SIZE") = MAZE_SIZE;

  /* Direction */
  py::class_<Direction> direction(m, "Direction");
  direction.attr("Max") = (int8_t)Direction::Max;
  direction.attr("ALong4") = py::cast(Direction::Along4);
  direction.attr("Diag4") = py::cast(Direction::Diag4);
  direction.def(py::init<int>())
      .def(py::init<enum Direction::AbsoluteDirection>())
      //   .def("Along4", &Direction::Along4)
      //   .def("Diag4", &Direction::Diag4)
      .def("__str__", &Direction::toChar)
      .def("__int__", [](const Direction& d) { return uint8_t(d); })
      .def(py::self == py::self)
      .def(py::self != py::self)
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
  py::implicitly_convertible<Direction::RelativeDirection, Direction>();
  py::bind_vector<Directions>(m, "Directions");

  /* Position */
  py::class_<Position> position(m, "Position");
  position.attr("SIZE") = (int)Position::SIZE;
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
           py::overload_cast<const Direction, const Position>(&Position::rotate,
                                                              py::const_),
           py::arg("d"), py::arg("center") = Position())
      .def("__str__",
           [](const Position& obj) {
             std::stringstream ss;
             ss << obj;
             return ss.str();
           })
      //
      ;
  py::bind_vector<Positions>(m, "Positions");

  /* Pose */
  py::class_<Pose>(m, "Pose")
      .def(py::init<Position&, Direction>(), py::arg("p") = Position(),
           py::arg("d") = Direction())
      .def_readwrite("p", &Pose::p)
      .def_readwrite("d", &Pose::d)
      .def("next", &Pose::next)
      .def("__str__",
           [](const Pose& obj) {
             std::stringstream ss;
             ss << obj;
             return ss.str();
           })
      //
      ;

  /* WallIndex */
  py::class_<WallIndex> wall_index(m, "WallIndex");
  wall_index.attr("SIZE") = (int)WallIndex::SIZE;
  wall_index
      //  .def(py::init<int8_t, int8_t, int8_t>(), py::arg("x") = 0,
      //       py::arg("y") = 0, py::arg("z") = 0)
      .def(py::init<int8_t, int8_t, int8_t>())
      .def(py::init<const Position&, const Direction>())
      //  .def(py::init<const uint16_t>())
      .def(py::init<const uint16_t>(), py::arg("index") = 0)
      .def(py::self == py::self)
      .def(py::self != py::self)
      .def("getIndex", &WallIndex::getIndex)
      .def("getPosition", &WallIndex::getPosition)
      .def("getDirection", &WallIndex::getDirection)
      .def("__str__",
           [](const WallIndex& obj) {
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
           py::arg("y") = 0, py::arg("d") = 0, py::arg("b") = false)
      .def(py::init<const Position&, const Direction, bool>())
      .def("getPosition", &WallRecord::getPosition)
      .def("getDirection", &WallRecord::getDirection)
      //
      ;
  py::class_<WallRecords>(m, "WallRecords");

  /* Maze */
  py::class_<Maze>(m, "Maze")
      .def(py::init<>())
      .def(py::init<const Positions&, const Position&>(),  //
           py::arg("goals") = Positions(), py::arg("start") = Position(0, 0))
      .def("reset", &Maze::reset, py::arg("set_start_wall") = true,
           py::arg("set_range_full") = false)
      /* isWall */
      .def("isWall",
           py::overload_cast<const WallIndex>(&Maze::isWall, py::const_))
      .def("isWall", py::overload_cast<const Position, const Direction>(
                         &Maze::isWall, py::const_))
      .def("isWall",
           py::overload_cast<const int8_t, const int8_t, const Direction>(
               &Maze::isWall, py::const_))
      /* setWall */
      .def("setWall", py::overload_cast<const WallIndex, bool>(&Maze::setWall))
      .def("setWall", py::overload_cast<const Position, const Direction, bool>(
                          &Maze::setWall))
      .def("setWall",
           py::overload_cast<const int8_t, const int8_t, const Direction, bool>(
               &Maze::setWall))
      /* isKnown */
      .def("isKnown",
           py::overload_cast<const WallIndex>(&Maze::isKnown, py::const_))
      .def("isKnown", py::overload_cast<const Position, const Direction>(
                          &Maze::isKnown, py::const_))
      .def("isKnown",
           py::overload_cast<const int8_t, const int8_t, const Direction>(
               &Maze::isKnown, py::const_))
      /* setKnown */
      .def("setKnown",
           py::overload_cast<const WallIndex, bool>(&Maze::setKnown))
      .def("setKnown", py::overload_cast<const Position, const Direction, bool>(
                           &Maze::setKnown))
      .def("setKnown",
           py::overload_cast<const int8_t, const int8_t, const Direction, bool>(
               &Maze::setKnown))
      /* canGo */
      .def("canGo",
           py::overload_cast<const WallIndex>(&Maze::canGo, py::const_))
      .def("canGo", py::overload_cast<const Position, const Direction>(
                        &Maze::canGo, py::const_))
      /* others */
      .def("updateWall",
           py::overload_cast<const Position, const Direction, bool, bool>(
               &Maze::updateWall),
           py::arg("p"), py::arg("d"), py::arg("b"), py::arg("pushLog") = true)
      .def("resetLastWalls", &Maze::resetLastWalls)
      .def("wallCount", &Maze::wallCount)
      .def("unknownCount", &Maze::unknownCount)
      /* print */
      .def("print",
           [](const Maze& maze) {
             py::scoped_ostream_redirect stream(
                 std::cout, py::module::import("sys").attr("stdout"));
             maze.print();
           })
      .def("print",
           [](const Maze& maze, const int maze_size) {
             py::scoped_ostream_redirect stream(
                 std::cout, py::module::import("sys").attr("stdout"));
             maze.print(std::cout, maze_size);
           })
      .def("print",
           [](const Maze& maze, const Directions& dirs, const Position& start,
              const int maze_size) {
             py::scoped_ostream_redirect stream(
                 std::cout, py::module::import("sys").attr("stdout"));
             maze.print(dirs, start, std::cout, maze_size);
           })
      /* parse */
      .def("parse", py::overload_cast<const std::string&>(&Maze::parse))
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

  /* StepMap */
  py::class_<StepMap>(m, "StepMap")
      .def(py::init<>())
      .def("calcShortestDirections",
           py::overload_cast<const Maze&, const bool, const bool>(
               &StepMap::calcShortestDirections),
           py::arg("maze"), py::arg("known_only") = true,
           py::arg("simple") = false)
      .def_static("appendStraightDirections",
                  &StepMap::appendStraightDirections, py::arg("maze"),
                  py::arg("directions"), py::arg("known_only"),
                  py::arg("diag_enabled"))
      //
      ;

  /* StepMapWall */
  py::class_<StepMapWall>(m, "StepMapWall")
      .def(py::init<>())
      .def("calcShortestDirections",
           py::overload_cast<const Maze&, const bool, const bool>(
               &StepMapWall::calcShortestDirections),
           py::arg("maze"), py::arg("known_only") = true,
           py::arg("simple") = false)
      .def_static("convertWallIndexDirectionsToPositionDirections",
                  &StepMapWall::convertWallIndexDirectionsToPositionDirections,
                  py::arg("src"), py::arg("start") = WallIndex(0, 0, 1))
      //
      ;

  /* StepMapSlalom */
  py::class_<StepMapSlalom> step_map_slalom(m, "StepMapSlalom");
  py::class_<StepMapSlalom::EdgeCost>(step_map_slalom, "EdgeCost")
      .def(py::init<>())
      //
      ;
  step_map_slalom
      .def(py::init<>())
      //  .def(
      //      "calcShortestDirections",
      //      [](StepMapSlalom &map, const Maze &maze, const bool known_only,
      //         const bool diag_enabled) {
      //        Directions shortest_dirs;
      //        StepMapSlalom::EdgeCost edge_cost;
      //        map.calcShortestDirections(maze, edge_cost, shortest_dirs,
      //                                   known_only, diag_enabled);
      //        return shortest_dirs;
      //      },
      //      py::arg("maze"), py::arg("known_only") = true,
      //      py::arg("diag_enabled") = true)
      .def(
          "calcShortestDirections",
          [](StepMapSlalom& map, const Maze& maze,
             const StepMapSlalom::EdgeCost& edge_cost, const bool known_only,
             const bool diag_enabled) {
            Directions shortest_dirs;
            map.calcShortestDirections(maze, edge_cost, shortest_dirs,
                                       known_only, diag_enabled);
            return shortest_dirs;
          },
          py::arg("maze"), py::arg("edge_cost") = StepMapSlalom::EdgeCost(),
          py::arg("known_only") = true, py::arg("diag_enabled") = true)
      //
      ;
}
