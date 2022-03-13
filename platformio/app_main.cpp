/**
 * @file app_main.cpp
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief MicroMouse Maze Library のマイコン上での実行時間計測コード
 * @date 2021-01-25
 * @copyright Copyright (c) 2021 Ryotaro Onuki
 */
#include <MazeLib/Maze.h> /*< for maze_logi */
#include <SPIFFS.h>
#include <iostream>

/* defined in examples/meas/main.cpp */
int test_meas(const std::string& mazedata_dir);

void setup() {
  SPIFFS.begin(true);
  vTaskDelay(pdMS_TO_TICKS(1000));
  maze_logi << "Hello, this is ESP32 :)" << std::endl;
}

void loop() {
  test_meas("/spiffs/");
  maze_logi << "End" << std::endl;
  vTaskDelay(portMAX_DELAY);
}
