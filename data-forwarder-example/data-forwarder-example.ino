#include <Wire.h>

#include "SparkFun_LIS2DH12.h" //Click here to get the library: http://librarymanager/All#SparkFun_LIS2DH12
SPARKFUN_LIS2DH12 accel;       //Create instance

#define FREQUENCY_HZ        50
#define INTERVAL_MS         (1000 / (FREQUENCY_HZ + 1))

void setup() {
    Serial.begin(115200);
    Serial.println("Accelerometer reading in Serial Console to be forwarded to Edge Impulse");

    Wire.begin();

    if (accel.begin() == false)
    {
      Serial.println("Accelerometer not detected. Check address jumper and wiring. Freezing...");
      while (1);
    }
    accel.setDataRate(LIS2DH12_ODR_100Hz);
}

void loop() {
    static unsigned long last_interval_ms = 0;
    
    if (accel.available() && millis() > last_interval_ms + INTERVAL_MS) {
        last_interval_ms = millis();
  
        Serial.print(accel.getX());
        Serial.print('\t');
        Serial.print(accel.getY());
        Serial.print('\t');
        Serial.println(accel.getY());   
    }
}
