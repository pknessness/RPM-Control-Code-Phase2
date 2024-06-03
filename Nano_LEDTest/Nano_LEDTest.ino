#define LED_BUILTIN 13

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  for(int i = 0; i < 5; i++) {
    digitalWrite(LED_BUILTIN, HIGH); // Turn the LED on
    delay(1000); // Wait for 1 second
    digitalWrite(LED_BUILTIN, LOW); // Turn the LED off
    delay(1000); // Wait for 1 second
  }
  while(1); // Stop the loop after blinking 5 times
}




