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

Maze sample(mazeData_MM2018HX);

int main(void) {
  setvbuf(stdout, (char *)NULL, _IONBF, 0);
  sample.setGoals({
      Vector(11, 11),
      Vector(11, 12),
      Vector(11, 13),
      Vector(12, 11),
      Vector(12, 12),
      Vector(12, 13),
      Vector(13, 11),
      Vector(13, 12),
      Vector(13, 13),
  });
  std::ofstream of("32MM2018HX.maze");
  sample.print(of);
  sample.print();
  return 0;
}
