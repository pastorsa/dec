/**
 *
 * Sample Multi Master I2C implementation.  Sends a button state over I2C to another
 * Arduino, which flashes an LED correspinding to button state.
 * 
 * Connections: Arduino analog pins 4 and 5 are connected between the two Arduinos, 
 * with a 1k pullup resistor connected to each line.  Connect a push button between 
 * digital pin 10 and ground, and an LED (with a resistor) to digital pin 9.
 * 
 */

#include <Wire.h>

#define LED 9
#define BUTTON 10

#define THIS_ADDRESS 0x9
#define OTHER_ADDRESS 0x8

boolean last_state = HIGH;

long rand_number;

void setup() {

  // if analog input pin 0 is unconnected, random analog
  // noise will cause the call to randomSeed() to generate
  // different seed numbers each time the sketch runs.
  // randomSeed() will then shuffle the random function.
  randomSeed(analogRead(0));



  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  
  pinMode(BUTTON, INPUT);
  digitalWrite(BUTTON, HIGH);
  
  Wire.begin(THIS_ADDRESS);
  Wire.onReceive(receiveEvent);
}

void loop() {
  
  // print a random number from 0 to 99
  rand_number = random(0,100);
  Serial.println(rand_number);  

  // print a random number from 10 to 19
  randNumber = random(10, 20);
  
  if (digitalRead(BUTTON) != last_state){
    last_state = digitalRead(BUTTON);
    Wire.beginTransmission(OTHER_ADDRESS);
    Wire.send(last_state);
    Wire.endTransmission();
  }
}

void receiveEvent(int howMany){
  while (Wire.available() > 0){
    boolean b = Wire.receive();
    Serial.print(b, DEC);
    digitalWrite(LED, !b);
  }
  Serial.println(); 
} 
