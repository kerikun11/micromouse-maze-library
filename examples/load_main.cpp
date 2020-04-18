#include "Maze.h"

using namespace MazeLib;

const std::vector<std::string> mazeData = {
    "6aaaaaaaaaaaaaa3", "56aaaaaaaaaaaa35", "556aaaaaaaaaa355",
    "5556aaaaaaaa3555", "55556aaaaaa35555", "555556aaaa355555",
    "5555556aa3555555", "5555555635555555", "5555555c81555555",
    "5555555ea9555555", "555555caaa955555", "55555caaaaa95555",
    "5555caaaaaaa9555", "555caaaaaaaaa955", "55caaaaaaaaaaa95",
    "dcaaaaaaaaaaaaa9",
};

int main(void) {
  std::cout << "Maze File Generator" << std::endl;
#if 1
  /* parameter */
  const auto maze_data = mazeData;
  const int maze_size = mazeData.size();
  const std::string output_filename = "output.maze";
  const Positions goals = {
      Position(7, 7), Position(8, 7), Position(7, 8), Position(8, 8),
      // Position(6, 9), Position(6, 10), Position(7, 9), Position(7, 10),
      // Position(4, 4), Position(5, 4), Position(4, 5), Position(5, 5),
  };
  /* process */
  Maze sample;
  std::ofstream of(output_filename);
  sample.parse(maze_data, maze_size);
  sample.setGoals(goals);
  sample.print(of, maze_size);
  sample.print(std::cout, maze_size);
#endif

#if 0
  const std::string mazedata_dir =
      "/home/kerikun11/Dropbox/Projects/MicroMouse/GitHub/micromouseonline/"
      "mazefiles/classic/";
  for (int y = 2015; y >= 2014; --y) {
    const std::string filename = "japan" + std::to_string(y) + "ef.txt";
    Maze maze = Maze((mazedata_dir + filename).c_str());
    maze.print(std::cout, maze.getMaxX() + 1);
    std::ofstream of("16MM" + std::to_string(y) + "CX.maze");
    maze.print(of, maze.getMaxX() + 1);
  }
#endif

#if 0
  /* Preparation */
  const std::string mazedata_dir =
      "/home/kerikun11/Dropbox/Projects/MicroMouse/GitHub/micromouseonline/"
      "mazefiles/classic/";
  const std::string filename = "alljapan-033-2012-exp-fin.txt";
  Maze maze = Maze((mazedata_dir + filename).c_str());
  maze.print(std::cout, maze.getMaxX() + 1);
  std::ofstream of(filename);
  maze.print(of, maze.getMaxX() + 1);
#endif
  return 0;
}
