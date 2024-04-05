#include "app/twiddlerino_main.h"

//twiddlerino entry point
void setup() {
  esp_log_level_set("*", ESP_LOG_ERROR);
  twiddlerino_setup();
}

void loop() {
  //leave empty
  //we are using free rtos tasks
}