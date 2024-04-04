

#include "app/twiddlerino_main.h"

void setup() {
  esp_log_level_set("*", ESP_LOG_VERBOSE);
  //everything is setup here
  twiddlerino_setup(startup_type_t::RUN_CONTROLLER_DEFAULT);
}

void loop() {
  //leave empty
  //we are using free rtos tasks
}