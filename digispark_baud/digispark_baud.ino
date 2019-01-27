#include <TinyPinChange.h>
#include <SoftSerial.h>
#include <util/atomic.h>
#include <avr/io.h>

SoftSerial mySerial(-1, 0);

void setup() {
  pinMode(1, OUTPUT);
  mySerial.begin(115200);
  digitalWrite(1, HIGH);
}

void loop() {
  mySerial.print("UUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUU");
}
