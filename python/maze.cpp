#include <Maze.h>

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

BOOST_PYTHON_MODULE(maze) {
  using namespace boost::python;
  using namespace MazeLib;

  scope().attr("MAZE_SIZE") = MAZE_SIZE;

  class_<Direction>("Direction", init<int>())
      .def(self_ns::str(self))
      .def(
          "getAlong4",
          +[]() {
            const auto &dirs = Direction::getAlong4();
            return Directions{dirs.cbegin(), dirs.cend()};
          })
      .def(
          "getDiag4",
          +[]() {
            const auto &dirs = Direction::getDiag4();
            return Directions{dirs.cbegin(), dirs.cend()};
          })
      //
      ;
  enum_<Direction::AbsoluteDirection>("AbsoluteDirection")
      .value("East", Direction::East)
      .value("NorthEast", Direction::NorthEast)
      .value("North", Direction::North)
      .value("NorthWest", Direction::NorthWest)
      .value("West", Direction::West)
      .value("SouthWest", Direction::SouthWest)
      .value("South", Direction::South)
      .value("SouthEast", Direction::SouthEast)
      //
      ;
  enum_<Direction::RelativeDirection>("RelativeDirection")
      .value("Front", Direction::Front)
      .value("Left45", Direction::Left45)
      .value("Left", Direction::Left)
      .value("Left135", Direction::Left135)
      .value("Back", Direction::Back)
      .value("Right135", Direction::Right135)
      .value("Right", Direction::Right)
      .value("Right45", Direction::Right45)
      //
      ;
  class_<Directions>("Directions").def(vector_indexing_suite<Directions>());

  class_<Position>("Position")
      .def_readwrite("x", &Position::x)
      .def_readwrite("y", &Position::y)
      .def("getIndex", &Position::getIndex)
      .def(self + self)
      .def(self - self)
      .def(self == self)
      .def(self != self)
      .def("next", &Position::next)
      .def("isInsideOfField", &Position::isInsideOfField)
      //   .def("rotate", &Position::rotate)
      .def(self_ns::str(self))
      //
      ;
  class_<Positions>("Positions").def(vector_indexing_suite<Positions>());

  //   class_<Pose>("Pose", init<Position &, Direction>())
  //       .def_readwrite("p", &Pose::p)
  //       .def_readwrite("d", &Pose::d)
  //       .def("next", &Pose::next)
  //       .def(self_ns::str(self))
  //       //
  //       ;

  //   class_<slalom::Shape>(
  //       "Shape",
  //       //   init<Pose, Pose, float, float, float, float, float, float>())
  //       init<Pose, float, float, float, float, float>())
  //       .def_readwrite("total", &slalom::Shape::total)
  //       .def_readwrite("curve", &slalom::Shape::curve)
  //       .def_readwrite("straight_prev", &slalom::Shape::straight_prev)
  //       .def_readwrite("straight_post", &slalom::Shape::straight_post)
  //       .def_readwrite("v_ref", &slalom::Shape::v_ref)
  //       .def_readwrite("dddth_max", &slalom::Shape::dddth_max)
  //       .def_readwrite("ddth_max", &slalom::Shape::ddth_max)
  //       .def_readwrite("dth_max", &slalom::Shape::dth_max)
  //       .def("integrate", &slalom::Shape::integrate)
  //       .def(self_ns::str(self))
  //       //
  //       ;

  //   class_<State>("State")
  //       .def_readwrite("q", &State::q)
  //       .def_readwrite("dq", &State::dq)
  //       .def_readwrite("ddq", &State::ddq)
  //       .def_readwrite("dddq", &State::dddq)
  //       //
  //       ;

  //   class_<slalom::Trajectory>("Trajectory", init<slalom::Shape &, bool>())
  //       .def("reset", &slalom::Trajectory::reset)
  //       .def(
  //           "update",
  //           +[](const slalom::Trajectory &obj, const State &s, float t, float
  //           Ts,
  //               float k_slip) {
  //             auto state = s;
  //             obj.update(state, t, Ts, k_slip);
  //             return state;
  //           })
  //       .def("getVelocity", &slalom::Trajectory::getVelocity)
  //       .def("getTimeCurve", &slalom::Trajectory::getTimeCurve)
  //       .def("getShape", &slalom::Trajectory::getShape,
  //            return_value_policy<copy_const_reference>())
  //       .def("getAccelDesigner", &slalom::Trajectory::getAccelDesigner,
  //            return_value_policy<copy_const_reference>())
  //       //
  //       ;
}
