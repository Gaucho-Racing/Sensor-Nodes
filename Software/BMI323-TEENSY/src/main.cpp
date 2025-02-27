#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
// #include <Adafruit_MLX90614.h>
#include <FlexCAN_T4.h>
#include <stdio.h>
#include <math.h>

volatile uint16_t data[36];
volatile boolean newData = false;


void spiRecieve(){
  SPI.beginTransaction(SPISettings(12000000, MSBFIRST, SPI_MODE0));
  for(int i = 0; i < 36; i++){
    data[i] = SPI.transfer16(0x0101);
  }
  // delay(1);
  SPI.endTransaction();
  newData = true;
}


const uint8_t ledPin = LED_BUILTIN;
void setup(){
  SPI.begin();
  Serial.begin(115200);
  delay(1000);
  Serial.println("Start");
  pinMode(9, INPUT);
  attachInterrupt(digitalPinToInterrupt(9), spiRecieve, FALLING);

}

void loop(){
if(newData == true){
  Serial.println("New Data");
  for(int i = 0; i < 36; i++){
    Serial.print(data[i]);
    Serial.print(" ");
  }
  newData = false;
}
}




void error(){
  while(1){
    //SOS
    //S
    digitalWrite(ledPin, HIGH);
    delay(200);
    digitalWrite(ledPin, LOW);
    delay(200);
    digitalWrite(ledPin, HIGH);
    delay(200);
    digitalWrite(ledPin, LOW);
    delay(200);
    digitalWrite(ledPin, HIGH);
    delay(200);
    digitalWrite(ledPin, LOW);
    delay(200);
    //O
    digitalWrite(ledPin, HIGH);
    delay(500);
    digitalWrite(ledPin, LOW);
    delay(200);
    digitalWrite(ledPin, HIGH);
    delay(500);
    digitalWrite(ledPin, LOW);
    delay(200);
    digitalWrite(ledPin, HIGH);
    delay(500);
    digitalWrite(ledPin, LOW);
    delay(200);
    //S
    digitalWrite(ledPin, HIGH);
    delay(200);
    digitalWrite(ledPin, LOW);
    delay(200);
    digitalWrite(ledPin, HIGH);
    delay(200);
    digitalWrite(ledPin, LOW);
    delay(200);
    digitalWrite(ledPin, HIGH);
    delay(200);
    digitalWrite(ledPin, LOW);
    delay(200);
    delay(2000);
  }
}



 