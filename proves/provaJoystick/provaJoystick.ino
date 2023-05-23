#include <Wire.h>

int xMap, xValue;
int analogPin = A3;
int outputPin1 = 4; // PWM output pin
int outputPin2 = 5; // PWM output pin
int outputPin3 = 6; // PWM output pin
int outputPin4 = 7; // PWM output pin
unsigned long timerValue;

void setup() {
  Serial.begin(115200);
  pinMode(outputPin1, OUTPUT);
  pinMode(outputPin2, OUTPUT);
  pinMode(outputPin3, OUTPUT);
  pinMode(outputPin4, OUTPUT);
}

void loop() {
  xValue = 1023;//analogRead(analogPin);
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
  writePins(HIGH);
  //digitalWrite(outputPin, HIGH);

  // Wait for the duration of the pulse
  while (micros() < timerValue + xMap);

  // Set the output pin LOW
  //digitalWrite(outputPin, LOW);
  writePins(LOW);

  // Wait for the second half of the period
  while (micros() - timerValue < 20000);

}

void writePins(int value){
  digitalWrite(outputPin1, value);
  digitalWrite(outputPin2, value);
  digitalWrite(outputPin3, value);
  digitalWrite(outputPin4, value);
}







