#include <ESP32Servo.h>
#include <TinyPICO.h>

// Initial led object
TinyPICO tp = TinyPICO();

// Number of servos
const int numServos = 6;

// Create array for servo obejcts
Servo servos[numServos];

// Define the pins for the servod
const int servoPins[numServos] = {33, 32, 4, 14, 15, 27};


void setup() {
  Serial.begin(115200);

  // Set Blu light for status
  tp.DotStar_SetPixelColor(0, 0, 255);

  // Attach servos to the pins
  for (int i = 0; i < numServos; i++) {
    servos[i].attach(servoPins[i]);    
    servos[i].write(90);
    delay(50);
  }

  delay(1000);
}

void loop() {
  // Status green for ready
  //tp.DotStar_SetPixelColor(0, 255, 0);

  // Check if data is available
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    data.trim();

    // Split the string into individual servo values
    int servoValues[numServos];
    int index = 0;
    int lastIndex = 0;

    for (int i = 0; i < data.length(); i++) {
      if (data.charAt(i) == ',') {
        String valueStr = data.substring(lastIndex, i);
        servoValues[index++] = valueStr.toInt();
        lastIndex = i + 1;
      }
    }
    // Add the last value after the final comma
    if (index < numServos) {
      String valueStr = data.substring(lastIndex);
      servoValues[index++] = valueStr.toInt();
    }

    // Ensure exactly 'numServos' values were received
    if (index == numServos) {
      for (int i = 0; i < numServos; i++) {
        servos[i].write(constrain(servoValues[i], 0, 180));
        delay(10);
      }       
    } else {
      tp.DotStar_SetPixelColor(254, 0, 0);
    }
  } else {
    tp.DotStar_SetPixelColor(255, 255, 0);
  }
}
