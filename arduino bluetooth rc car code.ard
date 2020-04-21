#I created an RC Bluetooth Car. I incorporated a ultrasonic senosr in my circuit to measure the distance away from a certain object and a voltmeter to test voltage outputs.


#include <Adafruit_BLE_UART.h>
#include <Adafruit_ATParser.h>
#include <Adafruit_BluefruitLE_SPI.h>
#include <Adafruit_BLEMIDI.h>
#include <Adafruit_BLEBattery.h>
#include <Adafruit_BLEGatt.h>
#include <Adafruit_BLEEddystone.h>
#include <Adafruit_BLE.h>
#include <Adafruit_BluefruitLE_UART.h>

#define AIN1 5
#define AIN2 6
#include <Stepper.h>

// MOTOR PURPOSES
#define Ain1 5
#define Ain2 6
#define Bin1 8
#define Bin2 7
int speed = 0;

// ultrasonic sensor (pin numbers)
const int trigPin = 4; // change later
const int echoPin = 3;

// defines variables
//long duration;
//int distance


// from echoDemo defines
/*********************************************************************
This is an example for our nRF8001 Bluetooth Low Energy Breakout

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/1697

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Kevin Townsend/KTOWN  for Adafruit Industries.
MIT license, check LICENSE for more information
All text above, and the splash screen below must be included in any redistribution
*********************************************************************/

// This version uses the internal data queing so you can treat it like Serial (kinda)!

#include <SPI.h>
#include "Adafruit_BLE_UART.h"

// Connect CLK/MISO/MOSI to hardware SPI
// e.g. On UNO & compatible: CLK = 13, MISO = 12, MOSI = 11
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 9

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);
/**************************************************************************/
/*!
    Configure the Arduino and start advertising with the radio
*/
/**************************************************************************/


// for analog voltmeter
int analogInput = 1;
float Vout = 0.00;
float Vin = 0.00;
float R1 = 100000.00; // resistance of R1 (100K) 
float R2 = 10000.00; // resistance of R2 (10K) 
int val = 0;

void setup() {
  
// input
//pinMode(1, OUTPUT);



// for the ultrasonic sensor 
pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
pinMode(echoPin, INPUT); // Sets the echoPin as an Input
Serial.begin(9600); // Starts the serial communication

// FOR MOTOR PURPOSES
pinMode(Ain1, OUTPUT);  //Ain1
pinMode(Ain2, OUTPUT);  //Ain2
pinMode(Bin1, OUTPUT);  //Bin1
pinMode(Bin2, OUTPUT);  //Bin2

//from voidsetup (void) echodemo class
  Serial.begin(9600);
  while(!Serial); // Leonardo/Micro should wait for serial init
  Serial.println(F("Adafruit Bluefruit Low Energy nRF8001 Print echo demo"));

  // BTLEserial.setDeviceName("NEWNAME"); /* 7 characters max! */

  BTLEserial.begin();

  // analog voltmeter to measure voltage
  pinMode(analogInput, INPUT); //assigning the input port
   Serial.begin(9600); //BaudRate

}


// echo demo void setup
aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

void loop() {

// ANALOG VOLTMETER

val = analogRead(analogInput);//reads the analog input
   Vout = (val * 5.00) / 1024.00; // formula for calculating voltage out i.e. V+, here 5.00
   Vin = Vout / (R2/(R1+R2)); // formula for calculating voltage in i.e. GND
   if (Vin<0.09)//condition 
   {
     Vin=0.00;//statement to quash undesired reading !
  } 
//Serial.print("\t Voltage of the given source = ");
//Serial.print(Vin);
delay(1000); //for maintaining the speed of the output in serial moniter

  // for the ultrasonic sensor
  // Clears the trigPin
 /* long duration;
  float distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration*0.034/2;
   //Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(distance);*/



// FOR MOTOR PURPOSES
 //   digitalWrite(Ain1,HIGH); // left wheel
 //   digitalWrite(Ain2,LOW);
 //   digitalWrite(Bin1,HIGH);
 //   digitalWrite(Bin2,LOW);
 //   delay(700); // 700 ms in between

    
   /* digitalWrite(Ain1,LOW); // left wheel
    digitalWrite(Ain2,LOW);
    digitalWrite(Bin1,LOW);
    digitalWrite(Bin2,LOW);
    digitalWrite(Ain1,LOW);
   digitalWrite(Ain2,HIGH);
    digitalWrite(Bin1,LOW);
    digitalWrite(Bin2,HIGH);
    delay(700);

    
    digitalWrite(Ain1,LOW); // left wheel
    digitalWrite(Ain2,LOW);
    digitalWrite(Bin1,LOW);
    digitalWrite(Bin2,LOW);*/


  // from echodemo void loop()
  
  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();

  // Ask what is our current status
  aci_evt_opcode_t status = BTLEserial.getState();
  // If the status changed....
  if (status != laststatus) {
    // print it out!
    if (status == ACI_EVT_DEVICE_STARTED) {
        Serial.println(F("* Advertising started"));
    }
    if (status == ACI_EVT_CONNECTED) {
        Serial.println(F("* Connected!"));
    }
    if (status == ACI_EVT_DISCONNECTED) {
        Serial.println(F("* Disconnected or advertising timed out"));
    }
    // OK set the last status change to this one
    laststatus = status;
  }

  if (status == ACI_EVT_CONNECTED) {
    // Lets see if there's any data for us!
    if (BTLEserial.available()) {
      // Lets see if there's any data for us!
    if (BTLEserial.available()) {
      Serial.println("The button pressed = ");
     // Serial.println("printing serial number");
     //Serial.print(BTLEserial.available());
     // Serial.println("end of serial number");
      //Serial.println(F(" bytes available from BTLE"));

    }
    }
    // OK while we still have something to read, get a character and print it out
    
   while (BTLEserial.available()) { // keeps going while available
    
      String c = BTLEserial.readString();
      Serial.println(c);

      if (c == "!B516"){ // forward
        Serial.print("direction: forward");
        digitalWrite(Ain1,LOW);
        digitalWrite(Ain2,HIGH); //the motors that makes it go fwd (left positive)
        digitalWrite(Bin2,LOW); 
        digitalWrite(Bin1,HIGH); // the motors that makes it go fwd (right positive)

      // for the ultrasonic sensor
      // Clears the trigPin
      long duration;
      float distance;
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      // Sets the trigPin on HIGH state for 10 micro seconds
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      // Reads the echoPin, returns the sound wave travel time in microseconds
      duration = pulseIn(echoPin, HIGH);
      // Calculating the distance
      distance = duration*0.034/2;
      //Prints the distance on the Serial Monitor
      Serial.print("Distance when moved fwd to the nearest object in mm: ");
      Serial.println(distance);
        
      }

      else if (c == "!B615"){ // backwards
        Serial.print("direction: backward");
        digitalWrite(Ain1,HIGH);
        digitalWrite(Ain2,LOW);
        digitalWrite(Bin2,HIGH);  
        digitalWrite(Bin1,LOW);

      // for the ultrasonic sensor
      // Clears the trigPin
      long duration;
      float distance;
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      // Sets the trigPin on HIGH state for 10 micro seconds
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      // Reads the echoPin, returns the sound wave travel time in microseconds
      duration = pulseIn(echoPin, HIGH);
      // Calculating the distance
      distance = duration*0.034/2;
      //Prints the distance on the Serial Monitor
      Serial.print("Distance when moved backward to the nearest object in mm: ");
      Serial.println(distance);
    
      }
      
      else if (c == "!B714"){ //LEFT is when only right wheel moves
        Serial.print("direction: left");
        digitalWrite(Ain1,HIGH);
        digitalWrite(Ain2,LOW);
        digitalWrite(Bin2,LOW); // only this one is going forward
        digitalWrite(Bin1,LOW);

        // for the ultrasonic sensor
      // Clears the trigPin
      long duration;
      float distance;
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      // Sets the trigPin on HIGH state for 10 micro seconds
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      // Reads the echoPin, returns the sound wave travel time in microseconds
      duration = pulseIn(echoPin, HIGH);
      // Calculating the distance
      distance = duration*0.034/2;
      //Prints the distance on the Serial Monitor
      Serial.print("Distance when moved to the left to the nearest object in mm: ");
      Serial.println(distance);
        
      }
      
    else if (c == "!B813"){ //RIGHT is when only left wheel moves
      
        Serial.print("right");
        digitalWrite(Ain1,LOW); // only this one is going forward
        digitalWrite(Ain2,LOW);
        digitalWrite(Bin2,HIGH); 
        digitalWrite(Bin1,LOW);

        // for the ultrasonic sensor
      // Clears the trigPin
      long duration;
      float distance;
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      // Sets the trigPin on HIGH state for 10 micro seconds
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      // Reads the echoPin, returns the sound wave travel time in microseconds
      duration = pulseIn(echoPin, HIGH);
      // Calculating the distance
      distance = duration*0.034/2;
      //Prints the distance on the Serial Monitor
      Serial.print("Distance when moved to the right to the nearest object in mm: ");
      Serial.println(distance);
      
      } 
      
  else if (c == "!B11:"){ //spin clockwise (left fwd, right back)
        Serial.print("spin clockwise");
        digitalWrite(Ain1,LOW); 
        digitalWrite(Ain2,HIGH); // LEFT FWD
        digitalWrite(Bin2,HIGH); 
        digitalWrite(Bin1,LOW);
      }

     else if (c == "!B219"){ //spin COUNTERclockwise (left back, right fwd)
        Serial.print("spin counterclockwise");
        digitalWrite(Ain1,HIGH);
        digitalWrite(Ain2,LOW);
        digitalWrite(Bin2,LOW); 
        digitalWrite(Bin1,HIGH);  
      }

      // this is for stopping the car
      else if (c == "!B507" || c == "!B606" || c == "!B705" || c == "!B804" || c == "!B10;" || c == "!B20:" || c == "!B309" || c == "!B408"){ //STOP
        digitalWrite(Ain1,LOW);
        digitalWrite(Ain2,LOW);
        digitalWrite(Bin2,LOW); 
        digitalWrite(Bin1,LOW);
      }


    // Next up, see if we have any data to get from the Serial console

    if (Serial.available()) {
      
      // Read a line from Serial
      Serial.setTimeout(500); // 100 millisecond timeout
      String s = Serial.readString();

      // We need to convert the line to bytes, no more than 20 at this time
      uint8_t sendbuffer[20];
      s.getBytes(sendbuffer, 20);
      char sendbuffersize = min(20, s.length());

      Serial.print(F("\n* Sending -> \"")); Serial.print((char *)sendbuffer); Serial.println("\"");

      // write the data
      BTLEserial.write(sendbuffer, sendbuffersize);
    }
  }
  }
}
