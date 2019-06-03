#include "Maze.h"

using namespace MazeLib;

const char mazeData_MM2018HX[32 + 1][32 + 1] = {
    "636aaaaaaaa36aaaaa2aaaaaaaaaaaa3", "55562aaaaa3dcaaaaa8aaaaaaaaaaa35",
    "555556222356aaaaaaaaa36aaaaa3615", "5c15540001556aaaaaaa3556aaa21555",
    "56955400015556aaaaa355556a215555", "5c35540001555562223555555f555555",
    "56015c8889415540001554888a955555", "5555caaaaa95554000155d6aaaa95555",
    "555caaaaaabd554000154a8a3eaa8815", "55caa363636295c8889556aa8aaaa355",
    "5576a954141569eaaaa1556aaaaa2155", "5555635555dd56aaaaa9d55622235555",
    "554141555cab556aaaaa355400015555", "5555c955caab5556aaa3555400015555",
    "555caa9caaab55556a35555400015555", "5556aaaaaaaa15555f55555c88895555",
    "5c956aaaaaaa9555ca81554aaaaa95d5", "563556aaaaabf49caab555caaa2aa835",
    "415555622236216aaaa95de2a3ca3755", "555555400014014aaaaa9ea835635555",
    "55555540001c895776aaaaa355415555", "555415400016375d4162223555415555",
    "555555c8889555c29d40001555c95555", "555dc8a2aaa954a96340001555e29555",
    "5556aaa8aaaa956b554000155568a955", "555caaaaaaa2355f55c8889555ca2a95",
    "55563776237555cb5caaaaa955620ab5", "5555541401555cabca2aaaaa94154ab5",
    "55d5555c8955caaaaa96aaaaa955cab5", "483555caaa9caaaaaaa97eaaaa956221",
    "43c948aaaaaaaaaaaa3e8aaaaaa95dd5", "dcaa8aaaaaaaaaaaaa8aaaaaaaaa8aa9",
};

const char mazeData_MM2016CX[16 + 1][16 + 1] = {
    "a6666663ba627a63", "c666663c01a43c39", "a2623b879847c399",
    "9c25c05b85e23999", "9a43a5b85e219999", "9c385b85e25d9999",
    "9e05b85e25a39999", "9a5b85ba1a599999", "99b85b84587c5999",
    "9c05b85a20666599", "c3db85a5d9bbbb99", "b87847c639800059",
    "85e466665c5dddb9", "8666666666666645", "c666666666666663",
    "e666666666666665",
};

const char mazeData_MM2018CM[16 + 1][16 + 1] = {
    "7776aaaaaaaaaaa3", "55416aaaaaaaaaa1", "c0155eaa36aaaa35",
    "695556aa096aaa15", "4a9554a283caa355", "4aa95568bd6aa955",
    "56aa940b63caa355", "54aaa9d614aaa9dd", "556aa37c942a2a23",
    "5c8a354aa9c20a15", "4aa355caaaa9ca15", "4a35556aaaaaaa81",
    "c35555caaaaaaa35", "60155caaaaaaaa9d", "5541caaaaaaaaaa3",
    "dddcaaaaaaaaaaa9",
};

const char mazeData_MM2018MS[16 + 1][16 + 1] = {
    "6aa2aa2aaa2aaaa3", "4be82bc2a296a235", "4ab683e828356155",
    "57e0a0be8a95c955", "c0bc296a2b682a95", "e0be0b4b56169621",
    "e0b6169695497555", "e0b54969e0161555", "6834969629555c81",
    "c35d696956955635", "6956969695695555", "c35ca8a969569541",
    "e1caaa36975c3555", "6177775c34969555", "5540080bc9e8a9c1",
    "dc9dca8aaaaaaaa9",
};

const char mazeData_MM2017Tashiro[16 + 1][16 + 1] = {
    "6b6a3f6a3f6a3f63", "d696969696969695", "69fca96969696969",
    "56a3f6969696969f", "c969696969696963", "f69f569696969695",
    "6963c9fca9696969", "5695f6363696969f", "c969695c15696963",
    "f69696969c969695", "696969696a356969", "56969696969c969f",
    "c969696969636963", "f696969696955695", "696969696969c969",
    "dfca9fca9fcaaa9f",
};

int main(void) {
  std::cout << "Maze File Generator" << std::endl;
  /* parameter */
  const auto maze_data = mazeData_MM2018HX;
  const std::string output_filename = "output.maze";
#define BIT_ORDER 0
#if BIT_ORDER == 0
  std::array<Dir, 4> bit_order = {Dir::East, Dir::North, Dir::West, Dir::South};
#elif BIT_ORDER == 1
  std::array<Dir, 4> bit_order = {Dir::North, Dir::East, Dir::South, Dir::West};
#endif
  const Vectors goals = {
      // Vector(7, 7),
      // Vector(8, 7),
      // Vector(7, 8),
      // Vector(8, 8),
      Vector(12, 12),
      Vector(13, 12),
      Vector(12, 13),
      Vector(13, 13),
  };
  /* process */
  Maze sample(maze_data, bit_order);
  sample.setGoals(goals);
  std::ofstream of(output_filename);
  sample.print(of);
  sample.print();
  return 0;
}
