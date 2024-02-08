
#include "app/twiddlerino.h"
#include "Arduino.h"

const startup_type_t mode = startup_type_t::RUN_CONTROLLER_DEFAULT;

void setup() {
  //everything is setup here
  twiddlerino_setup(mode);
}

void loop() {
  //leave empty
  //we are using free rtos tasks
}