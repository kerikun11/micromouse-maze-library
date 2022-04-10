/**
 * @file app_main.cpp
 * @brief MicroMouse Maze Library のマイコン上での実行時間計測コード
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-01-25
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#include <MazeLib/Maze.h> /*< for MAZE_LOGI */
#include <SPIFFS.h>
#include <esp_ota_ops.h>
#include <iostream>

/* defined in examples/meas/main.cpp */
int test_meas(const std::string& mazedata_dir, const std::string& save_dir);

void setup() {
  SPIFFS.begin(true);
  vTaskDelay(pdMS_TO_TICKS(3000));
  MAZE_LOGI << "Hello, this is ESP32 :)" << std::endl;
  esp_app_desc_t app_desc;
  const esp_partition_t* running = esp_ota_get_running_partition();
  ESP_ERROR_CHECK(esp_ota_get_partition_description(running, &app_desc));
  MAZE_LOGI << "version: " << app_desc.version << std::endl;
  MAZE_LOGI << "date: " << __DATE__ << std::endl;
}

void loop() {
  test_meas("/spiffs/", "/spiffs/");
  MAZE_LOGI << "End" << std::endl;
  vTaskDelay(portMAX_DELAY);
}
