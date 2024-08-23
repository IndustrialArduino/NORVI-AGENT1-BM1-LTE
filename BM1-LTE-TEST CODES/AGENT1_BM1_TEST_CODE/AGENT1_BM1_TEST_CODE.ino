/*
 * NORVI-AGENT-BM1-LTE
 */
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads1;
#define VOLTAGE_DIVIDER_RATIO 0.4005

#define BUTTON_PIN   35    // Digital IO pin connected to the button.  This will be

#define LED_PIN 25     // Pin connected to the data input of the LED
#define NUM_LEDS 1     // Number of LEDs (1 in this case)

#define OUTPUT1 23 
#define OUTPUT2 13

#define RXD 23 // 485 DIFINE
#define TXD 13
#define FC  32

#define GSM_RX 4
#define GSM_TX 5
#define GSM_RESET 19

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() { 
  Serial.begin(115200);   

  pinMode(GSM_RESET, OUTPUT);
  digitalWrite(GSM_RESET, HIGH);   // GSM
  
  pinMode(FC, OUTPUT); 
  digitalWrite(FC, HIGH);
  
  Serial2.begin(115200, SERIAL_8N1, GSM_RX, GSM_TX); 
  Serial1.begin(9600, SERIAL_8N1,RXD,TXD);

  //LED CHECK
  pinMode(27, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(18, OUTPUT);
  
  strip.begin();
  strip.show(); 

  delay(100);
  input_led_test();
  delay(500);
  input_led_test();
  delay(500);
  rgb_test();
  delay(1000);
  
  pinMode(27, INPUT);
  pinMode(26, INPUT);
  pinMode(18, INPUT); 
 
  pinMode(BUTTON_PIN, INPUT);
  
   Wire.begin(21,22);
   
  if (!ads1.begin(0x48)) {
    Serial.println("Failed to initialize ADS 1 .");
    while (1);
  }
  ads1.setGain(GAIN_ONE);  // 1x gain +/- 4.096V  (1 bit = 0.125mV)
 digitalWrite(FC, HIGH);   // RS-485 
}

void loop() {
  int16_t adc0, adc1, adc2, adc3;
  Serial.println("");    
  Serial.print("BUTTON: ");Serial.println(digitalRead(BUTTON_PIN));

  Serial.print("I1: ");Serial.println(digitalRead(27));
  Serial.print("I2: ");Serial.println(digitalRead(26));
  Serial.print("I3: ");Serial.println(digitalRead(18));
  delay(200);
  
  adc0 = ads1.readADC_SingleEnded(0);
  adc1 = ads1.readADC_SingleEnded(1);
  adc2 = ads1.readADC_SingleEnded(2);
  adc3 = ads1.readADC_SingleEnded(3);
 
  float voltage0 = adc0 * 0.125 / 1000.0 / VOLTAGE_DIVIDER_RATIO;  
  float voltage1 = adc1 * 0.125 / 1000.0 / VOLTAGE_DIVIDER_RATIO; 
  float voltage2 = adc2 * 0.125 / 1000.0 / VOLTAGE_DIVIDER_RATIO;  
  float voltage3 = adc3 * 0.125 / 1000.0 / VOLTAGE_DIVIDER_RATIO; 
  
  Serial.print("Input Voltage 0: "); Serial.print(voltage1); Serial.println(" V");
  Serial.print("Input Voltage 1: "); Serial.print(voltage2); Serial.println(" V");
  Serial.print("Input Voltage 2: "); Serial.print(voltage0); Serial.println(" V");
 
 while (Serial.available()) {
    int inByte = Serial.read();
    Serial2.write(inByte);
  }

  while (Serial2.available()) {
    int inByte = Serial2.read();
    Serial.write(inByte);
  }
  Serial.println("");
  delay(100);
 
  digitalWrite(FC, HIGH);                    // Make FLOW CONTROL pin HIGH
  delay(300);
  Serial1.println(F("RS485 01 SUCCESS"));    // Send RS485 SUCCESS serially
  delay(300);                                // Wait for transmission of data
  digitalWrite(FC, LOW) ;                    // Receiving mode ON
  delay(300);     
  
  while (Serial1.available()) {  // Check if data is available
    char c = Serial1.read();     // Read data from RS485
    Serial.write(c);             // Print data on serial monitor
  }
 delay(300); 
 Serial.println("____________________________________");  
}

void input_led_test(){
  digitalWrite(27, LOW); 
  digitalWrite(26, HIGH);
  digitalWrite(18, HIGH);   

  delay(500);
  digitalWrite(27, HIGH); 
  digitalWrite(26, LOW);
  digitalWrite(18, HIGH);   
  
  delay(500);
  digitalWrite(27, HIGH); 
  digitalWrite(26, HIGH);
  digitalWrite(18, LOW);   

  delay(500);
  digitalWrite(27, HIGH); 
  digitalWrite(26, HIGH);
  digitalWrite(18, HIGH);   
   
  delay(500);
  digitalWrite(27, HIGH); 
  digitalWrite(26, HIGH);
  digitalWrite(18, HIGH);    
}

void rgb_test(){
  strip.setPixelColor(0, strip.Color(255, 0, 0)); // Red
  strip.show();
  delay(1000); // Wait for 1 second

  strip.setPixelColor(0, strip.Color(0, 255, 0)); // Green
  strip.show();
  delay(1000); // Wait for 1 second

  strip.setPixelColor(0, strip.Color(0, 0, 255)); // Blue
  strip.show();
  delay(1000); // Wait for 1 second

  strip.setPixelColor(0, strip.Color(255, 255, 0)); // Yellow
  strip.show();
  delay(1000); // Wait for 1 second

  strip.setPixelColor(0, strip.Color(0, 255, 255)); // Cyan
  strip.show();
  delay(1000); // Wait for 1 second

  strip.setPixelColor(0, strip.Color(255, 0, 255)); // Magenta
  strip.show();
  delay(1000); // Wait for 1 second

  strip.setPixelColor(0, strip.Color(255, 255, 255)); // White
  strip.show();
  delay(1000); // Wait for 1 second

  strip.setPixelColor(0, strip.Color(0, 0, 0)); // Off
  strip.show();
  delay(1000); // Wait for 1 second
}
