#include <Wire.h>

int xMap, xValue;
int analogPin = A3;
int outputPin = 9; // PWM output pin
unsigned long timerValue;

void setup() {
  Serial.begin(115200);
  pinMode(outputPin, OUTPUT);
}

void loop() {
  xValue = analogRead(analogPin);
  xMap = map(xValue, 0, 1023, 1000, 2010);
  Serial.print("X: ");
  Serial.print(xValue);
  Serial.print(", ");
  Serial.print("MAP: ");
  Serial.println(xMap);

  // Wait for the first half of the period
  while (micros() - timerValue < 10000);
  timerValue = micros();

  // Set the output pin HIGH
  digitalWrite(outputPin, HIGH);

  // Wait for the duration of the pulse
  while (micros() < timerValue + xMap);

  // Set the output pin LOW
  digitalWrite(outputPin, LOW);

  // Wait for the second half of the period
  while (micros() - timerValue < 20000);

}







