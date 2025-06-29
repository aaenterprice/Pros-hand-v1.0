#include <Servo.h>

Servo thumb, index, middle, ring, pinky;

int servoPins[] = {7, 8, 9, 10, 11};

Servo servos[] = {thumb, index, middle, ring, pinky};

String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(9600);

  for (int i = 0; i < 5; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(90); 
  }
}

void loop() {
  if (stringComplete) {
    if (inputString.startsWith("$")) {
      inputString.remove(0, 1); 

      int angles[5]; 
      int index = 0;

      char *ptr = strtok(inputString.c_str(), ",");
      while (ptr != NULL && index < 5) {
        angles[index] = atoi(ptr);
        ptr = strtok(NULL, ",");
        index++;
      }

      for (int i = 0; i < 5; i++) {
        int correctedAngle = constrain(angles[i], 0, 180);
        servos[i].write(correctedAngle);
      }
    }

    inputString = "";
    stringComplete = false;
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;

    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
