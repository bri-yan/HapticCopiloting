
#include "app/twiddlerino.h"
#include "Arduino.h"
#include "Adafruit_ADS1X15.h"
#include "SPI.h"

const startup_type_t mode = startup_type_t::RUN_CONTROLLER_DEFAULT;

Adafruit_ADS1115 ads;

void setup() {
  Serial.begin(9600);
  
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }

  ads.startADCReading(MUX_BY_CHANNEL[0], /*continuous=*/false);
}

void loop()
{
  // If we don't have new data, skip this iteration.
  if (ads.conversionComplete()) {
    int16_t results = ads.getLastConversionResults();
    float volts = ads.computeVolts(results);

    Serial.print(volts); Serial.println("V");

    // Start another conversion.
    ads.startADCReading(MUX_BY_CHANNEL[0], /*continuous=*/false);
  }

  delay(10);
}