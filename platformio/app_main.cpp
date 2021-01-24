/**
 * @file app_main.cpp
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief MicroMouse Maze Library のマイコン上での実行時間計測コード
 * @date 2021-01-25
 * @copyright Copyright (c) 2021 Ryotaro Onuki
 */
#include <Maze.h> /*< for logi */
#include <SPIFFS.h>
#include <iostream>

/* defined in examples/meas/main.cpp */
int test_meas(const std::string &mazedata_dir);

void setup() {
  SPIFFS.begin(true);
  logi << "Hello, this is ESP32 :)" << std::endl;
}

void loop() {
  test_meas("/spiffs/");
  logi << "End" << std::endl;
  vTaskDelay(portMAX_DELAY);
}
