#define RXD2 4
#define TXD2 2

#define PIN  19
#define RES  5

void setup() {
// initialize both serial ports:
Serial.begin(115200);
Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
pinMode(PIN,OUTPUT);
pinMode(RES,OUTPUT);

digitalWrite(RES,HIGH);

digitalWrite(PIN,LOW);
delay(2000);
digitalWrite(PIN,HIGH);
}

void loop() {
  // read from port 1, send to port 0:
  if (Serial2.available()) {
    int inByte = Serial2.read();
    Serial.write(inByte);
  }
  // read from port 0, send to port 1:
  if (Serial.available()) {
    int inByte = Serial.read();
    Serial2.write(inByte);
  }
}
