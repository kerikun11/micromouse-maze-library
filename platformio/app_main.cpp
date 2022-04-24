/**
 * @file app_main.cpp
 * @brief MicroMouse Maze Library のマイコン上での実行時間計測コード
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-01-25
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#include <MazeLib/Maze.h>  //< for MAZE_LOGI
#include <esp_ota_ops.h>
#include <esp_spiffs.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <soc/rtc.h>  //< for rtc_clk_cpu_freq_get_config

/* defined in examples/meas/main.cpp */
int test_meas(const std::string& mazedata_dir, const std::string& save_dir);

/* mount SPIFFS */
bool mount_spiffs(const char* mount_point = "/spiffs") {
  esp_vfs_spiffs_conf_t conf = {
      .base_path = mount_point,
      .partition_label = NULL,
      .max_files = 5,
      .format_if_mount_failed = true,
  };
  ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));
  return true;
}

static int get_cpu_freq_in_mhz() {
  rtc_cpu_freq_config_t conf;
  rtc_clk_cpu_freq_get_config(&conf);
  return conf.freq_mhz;
}
static char* get_app_version() {
  static esp_app_desc_t app_desc;
  const esp_partition_t* running = esp_ota_get_running_partition();
  ESP_ERROR_CHECK(esp_ota_get_partition_description(running, &app_desc));
  return app_desc.version;
}

/* called by arduino */
void setup() {
  vTaskDelay(pdMS_TO_TICKS(3000));
  MAZE_LOGI << "Hello, this is ESP32." << std::endl;
  MAZE_LOGI << "CPU Freq: " << get_cpu_freq_in_mhz() << " MHz" << std::endl;
  MAZE_LOGI << "build timestamp: " << __DATE__ << " " << __TIME__ << std::endl;
  MAZE_LOGI << "version: " << get_app_version() << std::endl;

  mount_spiffs();
  test_meas("/spiffs/", "/spiffs/");
  MAZE_LOGI << "End" << std::endl;
}
void loop() {}

/* called by esp-idf */
#if 0
extern "C" __attribute__((weak)) int app_main() {
  setup();
  while (1) {
    loop();
    vPortYield();
  }
}
#endif
