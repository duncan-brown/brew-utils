#include <Arduino.h>

const int pinSwPod = 0;

int readSwitchpod() {
  int values[] = {-1,421,463,507,549,595,640,680,717,751,895,1023};
  analogReference(DEFAULT);
  int in = analogRead(pinSwPod);
  int out = -1;
  int result = -1;
  for (int i=0;i<=10;i++) { if ((in>values[i]) && (in<=values[i+1])) { out = i; } }
  if (out == 10) { out = -1; }
  if ((out >= 0) && (out <= 9)) { result = out; return result; }
  return result;
}

int readDebouncedSwitchpod() {
  int key = readSwitchpod();
  unsigned long storedMillisDebounce = millis();
  unsigned long storedMillisTimeout = millis();
  while (true) {
    int lastKey = key;
    key = readSwitchpod();
    if (lastKey != key) { storedMillisDebounce = millis(); }
    if ((millis() - storedMillisDebounce) > 20) break;
    if ((millis() - storedMillisTimeout ) > 150) { key = -1; break; }
  }
  return key;
}

void setup() {
  Serial.begin(9600);
}

int n=0;
bool stoProcessando = false;
int aCapo = 0;

void processaTasto(int key) {
  aCapo++;
  Serial.print(key);
  Serial.print("\n");
  if ((aCapo % 20) == 0) Serial.println();
}

void loop() {
  int key = readDebouncedSwitchpod();
  if (key == -1) { stoProcessando = false; }
  else {
    if (stoProcessando == false) {
      processaTasto(key);
      stoProcessando = true;
    }
  }
}

