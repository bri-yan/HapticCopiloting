
#include "app/twiddlerino.h"

void setup() {
  //everything is setup here
  twiddlerino_setup(startup_type_t::RUN_TCONTROL_DEFAULT);
}

void loop() {
  //leave empty
  //we are using free rtos tasks
}