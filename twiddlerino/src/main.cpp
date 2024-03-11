
#include "app/twiddlerino.h"
#include "Arduino.h"

//twiddler pin definitions
#include "app/twid32_pin_defs.h"

//drivers
#include "drivers/encoder.h"

void setup() {
  //everything is setup here
  twiddlerino_setup(startup_type_t::RUN_TCONTROL_DEFAULT);
}

void loop() {
  //leave empty
  //we are using free rtos tasks
}