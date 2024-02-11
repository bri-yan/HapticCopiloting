
#include "app/twiddlerino.h"
#include "Arduino.h"

//twiddler pin definitions
#include "app/twid32_pin_defs.h"

//drivers
#include "drivers/encoder.h"

// const startup_type_t mode = startup_type_t::RUN_CONTROLLER_DEFAULT;
void setup() {
  //everything is setup here
  twiddlerino_setup(startup_type_t::RUN_AWAIT_COMMANDS);
}

void loop() {
  //leave empty
  //we are using free rtos tasks
}