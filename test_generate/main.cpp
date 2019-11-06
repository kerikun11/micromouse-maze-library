#include "Maze.h"

using namespace MazeLib;

const std::vector<std::string> mazeData = {
    "636a2aa2a2a2aa2b", "5c160aa028a0aa03", "c3c9c369caa9e215",
    "682a3c9636aa2155", "c35696a954a21555", "695569635c214015",
    "c349c289c2155555", "6943e0be29c88895", "c355e0b616363635",
    "6955e169d5555555", "4349e88aa9c9c955", "5c0a363636363615",
    "4b43c9c9c9c9c9d5", "4b55636377777775", "4340141400000001",
    "dc9dc9c9dddddddd",

};

int main(void) {
  std::cout << "Maze File Generator" << std::endl;
#if 1
  /* parameter */
  const auto maze_data = mazeData;
  const int maze_size = mazeData.size();
  const std::string output_filename = "output.maze";
  const Positions goals = {
      Position(6, 9), Position(6, 10), Position(7, 9), Position(7, 10),
      // Position(7, 7), Position(8, 7), Position(7, 8), Position(8, 8),
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
      "/home/kerikun11/Dropbox/Projects/MicroMouse/Github/micromouseonline/"
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
      "/home/kerikun11/Dropbox/Projects/MicroMouse/Github/micromouseonline/"
      "mazefiles/classic/";
  const std::string filename = "alljapan-033-2012-exp-fin.txt";
  Maze maze = Maze((mazedata_dir + filename).c_str());
  maze.print(std::cout, maze.getMaxX() + 1);
  std::ofstream of(filename);
  maze.print(of, maze.getMaxX() + 1);
#endif
  return 0;
}
