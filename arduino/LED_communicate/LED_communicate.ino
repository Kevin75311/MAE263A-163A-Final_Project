// Map LED numbers to pins
const int ledPins[] = {9, 10, 11, 12, 13};  
// LED 1 -> 9
// LED 2 -> 10
// LED 3 -> 11
// LED 4 -> 12
// LED 5 -> 13

void setup() {
  Serial.begin(9600);

  for (int i = 0; i < 5; i++) {
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LOW);
  }


  for (int i = 0; i < 5; i++) {
    digitalWrite(ledPins[i], HIGH);
  }
  delay(1000);
  for (int i = 0; i < 5; i++) {
    digitalWrite(ledPins[i], LOW);
  }

  Serial.println("Arduino Ready");
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    // Expected formats:
    // "LED 1 ON"
    // "LED 3 OFF"
    //
    // Split into tokens
    int firstSpace = command.indexOf(' ');
    int secondSpace = command.indexOf(' ', firstSpace + 1);

    if (firstSpace == -1 || secondSpace == -1) return;

    String label = command.substring(0, firstSpace);   // "LED"
    int ledNum = command.substring(firstSpace + 1, secondSpace).toInt();
    String action = command.substring(secondSpace + 1); // "ON" or "OFF"

    if (label != "LED" || ledNum < 1 || ledNum > 5) return;

    int pin = ledPins[ledNum - 1];

    if (action == "ON") {
      digitalWrite(pin, HIGH);
      Serial.print("LED ");
      Serial.print(ledNum);
      Serial.println(" ON");
    }
    else if (action == "OFF") {
      digitalWrite(pin, LOW);
      Serial.print("LED ");
      Serial.print(ledNum);
      Serial.println(" OFF");
    }
  }
}
