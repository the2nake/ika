#include <Arduino.h>

int counter = 0;

int pot_base = A0;
int pot_lower = A1;
int pot_upper = A2;

long oversample(int pin, int bits = 0);

void setup() {
  Serial.begin(115200);

  pinMode(pot_base, INPUT);
  pinMode(pot_lower, INPUT);
  pinMode(pot_upper, INPUT);
}

void loop() {
  ++counter;
  Serial.println(String("BASE")  + oversample(pot_base,  3));
  Serial.println(String("LOWER") + oversample(pot_lower, 3));
  Serial.println(String("UPPER") + oversample(pot_upper, 3));
  delay(5);
}

long oversample(int pin, int bits) {
  long sum = 0;
  for (int i = 0; i < int(pow(4, bits)); ++i) {
    sum += analogRead(pin);
  }
  return (sum >> bits);
}