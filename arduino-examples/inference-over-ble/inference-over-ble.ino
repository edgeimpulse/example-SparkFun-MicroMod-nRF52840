/* Edge Impulse examples
 * Copyright (c) 2021 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* Includes ---------------------------------------------------------------- */
#include <Wire.h>
#include "SparkFun_LIS2DH12.h" //Click here to get the library: http://librarymanager/All#SparkFun_LIS2DH12
SPARKFUN_LIS2DH12 accel;       //Create instance

#include <ArduinoBLE.h>

// User defined service, using python to generate uuid: 
// >>> import uuid
// >>> uuid.uuid5(uuid.NAMESPACE_DNS, 'edgeimpulse.com')
BLEService inferenceService("2d0a515f-e0f7-5fc8-b50f-05e267afeb67");  
BLEStringCharacteristic inferenceCharacteristic("2d0a515f-e0f7-5fc8-b50f-05e267afeb67", BLERead | BLENotify, 56); // remote clients will be able to get notifications if this characteristic changes

/* Include Edge Impulse Library -------------------------------------------------------- 
 * Modify the following line according to your project name
 * Do not forget to import the library using "Sketch">"Include Library">"Add .ZIP Library..."
 * You can find one library version in this repository under ei-library/ei-fitness-classifier-arduino.zip 
 * that has been generated from this public project https://studio.edgeimpulse.com/public/36037/latest
 */
 #include <Fitness_classifier_inferencing.h>


/* Private variables ------------------------------------------------------- */
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal

/**
* @brief      Arduino setup function
*/
void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    delay(2000);
    Serial.println("Edge Impulse Inferencing Demo \nusing SparkFun MicroMod Machine Learning Carrier Board + nRF52840 Processor Board ");

    Serial.print("EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE: ");
    Serial.println(EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);

    Serial.print("EI_CLASSIFIER_INTERVAL_MS: ");
    Serial.println(EI_CLASSIFIER_INTERVAL_MS);

    Wire.begin();

    if (accel.begin() == false)
    {
      Serial.println("Accelerometer not detected. Check address jumper and wiring. Freezing...");
      while (1);
    }else{
      accel.setScale(LIS2DH12_2g);
//      Serial.print("Accelerometer scale: ");
//      Serial.println(accel.getScale());

      accel.setMode(LIS2DH12_LP_8bit);
//      Serial.print("Accelerometer mode: ");
//      Serial.println(accel.getMode());

      accel.setDataRate(LIS2DH12_ODR_100Hz);
//      Serial.print("Accelerometer data rate: ");
//      Serial.println(accel.getDataRate());
    }

    if (!BLE.begin()) {   // initialize BLE
      Serial.println("starting BLE failed!");
      while (1);
    }

    BLE.setLocalName("TinyML-coach");  // Set name for connection
    BLE.setAdvertisedService(inferenceService); // Advertise service
    inferenceService.addCharacteristic(inferenceCharacteristic); // Add characteristic to service
    BLE.addService(inferenceService); // Add service
    inferenceCharacteristic.writeValue("inference"); // Set first value string
  
    BLE.advertise();  // Start advertising
    Serial.print("Peripheral device MAC: ");
    Serial.println(BLE.address());
    Serial.println("Waiting for connections...");
}

/**
* @brief      Printf function uses vsnprintf and output using Arduino Serial
*
* @param[in]  format     Variable argument list
*/
void ei_printf(const char *format, ...) {
   static char print_buf[1024] = { 0 };

   va_list args;
   va_start(args, format);
   int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
   va_end(args);

   if (r > 0) {
       Serial.write(print_buf);
   }
}

/**
* @brief      Get data and run inferencing
*
* @param[in]  debug  Get debug info if true
*/
void loop()
{   

    // wait for a BLE central
  BLEDevice central = BLE.central();

  String inferenceResult = "";

  // if a central is connected to the peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's BT address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.print("Switching on accelerometer... ");
    accel.setDataRate(LIS2DH12_ODR_100Hz);
    Serial.println("Done");

    // check the battery level every 200ms
    // while the central is connected:
    while (central.connected()) {
        ei_printf("\n ---------------- \n Starting inferencing...\n");   
        ei_printf("Allocating buffer size...");
    
        // Allocate a buffer here for the values we'll read from the IMU
        float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };
    
        ei_printf("Done\n");
    
        ei_printf("Sampling...");
    
        static unsigned long acquisition_start = millis();
        
        for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += 3) {
            // Determine the next tick (and then sleep later)
            uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);
    
            // read to the end of the buffer
            buffer[ix + 0] = accel.getX();
            buffer[ix + 1] = accel.getY();
            buffer[ix + 2] = accel.getZ();
    
            delayMicroseconds(next_tick - micros());
        }
    
        static unsigned long acquisition_end = millis();
        static unsigned long acquisition_time = acquisition_end - acquisition_start;
    
        ei_printf("Done in %d ms\n", acquisition_time);
    
        // Turn the raw buffer in a signal which we can the classify
        signal_t signal;
        int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
        if (err != 0) {
            ei_printf("Failed to create signal from buffer (%d)\n", err);
            return;
        }
        ei_printf("Done...\n");
    
        ei_printf("Running the classifier... \n");
        ei_impulse_result_t result = { 0 };
    
        err = run_classifier(&signal, &result, debug_nn);
        if (err != EI_IMPULSE_OK) {
            ei_printf("ERR: Failed to run classifier (%d)\n", err);
            return;
        }
    
        // print the predictions
        ei_printf("Predictions ");
        ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
            result.timing.dsp, result.timing.classification, result.timing.anomaly);
        ei_printf(": \n");
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
            if(result.classification[ix].value >0.7 && inferenceResult != result.classification[ix].label){
              inferenceResult = result.classification[ix].label;
              sendInferenceOverBLE(inferenceResult);
            } 
        }
        Serial.print("Movement predicted: ");
        Serial.println(inferenceResult);
        
    }
    // when the central disconnects, turn off the LED:
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
    Serial.print("Switching off accelerometer to save battery... ");
    accel.setDataRate(LIS2DH12_POWER_DOWN);
    Serial.println("Done");
  }
    
    
}

void sendInferenceOverBLE(String inferenceResult) {
     // when the central disconnects, turn off the LED:
     Serial.println("Sending inference result over BLE..."); // print it
     inferenceCharacteristic.writeValue(inferenceResult); // Write value
        
}
