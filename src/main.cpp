/*****************************************************************
* Tracks capacity, voltage and current from 2 100Ahr LiFePO4
* batteries connected in parallel to form a power bank.
* Also switches a charge input to control max. charge / max. voltage.
*
* Using two INA226 chips and two external shunts the current flow into
* and out of each battery can be monitorred. Tracking the charge and
* discharge into each battery is more accurate and can alert to battery
* mismatch.
* 
* Work in progress of changing to use  INA226_M class based on
* a modified library for the  INA226 Current and Power
* Sensor Module originally Written by Wolfgang (Wolle) Ewald
*
*
*
******************************************************************/

#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include "battery.h"

constexpr int battery_a_address = 0x40;
constexpr int battery_b_address = 0x41;

//function declarations:
long scale(long, long);
void averageAnalogue(int, int);
void logCharge();

//addresses for each input defined above
const int analogIn[] = {A0}; 

// Output pins via driver
const int outPin1 = 9;  // Analog output pin that the LED is attached
const int outPin2 = 7;
const int outPin3 = 8;
// Analogue output pins
const int analogOutPin1 = 5;  // Analog output pin - direct output 

constexpr  int16_t  epromID = 2346;
constexpr float aref = 1.077;         // ref voltage for 1023 full scale conversion

//calibration voltage measurement
constexpr float voltBatteryRatio = 0.0445509;
constexpr long  milliBatteryVoltRange = aref / voltBatteryRatio  * 1000.0;


unsigned long  currentMillis = 0;
unsigned long  previousMillis = 0;
unsigned long  lapsedMillis = 0;
unsigned long  previousZeroMillis = 0;
unsigned long  lapsedZeroMillis = 0;
double lapsed = 0.0;


long batteryVolts = 0;

long sensorTotal[] = {0};
int outputValue = 0;  // value output to the PWM (analog out)

int indexIn;
int total = 200; //default number of reads to average over current readings

int out1Status = 0; 


byte mask = 0;
unsigned int dataAddress = 3;
byte timeCount = 0;
float lastLapsed = 0;
double lapsedHrs = 0;


Battery battery_a =  Battery();
Battery battery_b =  Battery();

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  for (uint16_t indexIn = 0; indexIn < sizeof(analogIn); indexIn++) {
    pinMode(analogIn[indexIn], INPUT); 
  }
  pinMode(analogOutPin1, OUTPUT);
  pinMode(outPin1, OUTPUT);
  pinMode(outPin2, OUTPUT);
  pinMode(outPin3, OUTPUT);

  digitalWrite(outPin1, HIGH);
  digitalWrite(outPin2, HIGH);
  digitalWrite(outPin3, HIGH);

  analogReference(INTERNAL);
  Wire.begin();

  if(!battery_a.setUpINA266(battery_a_address)){
    Serial.println("Failed to init INA226 for battery A. Check your wiring.");
    while(1){}
  }
  if(!battery_b.setUpINA266(battery_b_address)){
    Serial.println("Failed to init INA226 for battery B. Check your wiring.");
    while(1){}
  }
  Serial.println("init INA226 for battery A and B DONE"); 

  delay(10000);

  analogWrite(analogOutPin1, 0);
  int16_t  x;
  EEPROM.get(0, x);
  if ( x != epromID){
      Serial.print("Blanking EEPROM :");
      Serial.println(EEPROM.length());
      for (uint16_t i = 2 ; i < EEPROM.length() ; i++) {
        EEPROM.write(i, 0);
      }
      x = epromID;
      EEPROM.put(0, x);
      mask = 255;
      EEPROM[2] = mask;
      dataAddress = 3;
      timeCount = 0;
      battery_a.charge = maxCapacity;
      battery_b.charge = maxCapacity;
      logCharge();
  } else {
      //need to find last entry where mask changes
      byte start = EEPROM[2] & 128;
      for (uint16_t i = 6 ; i < EEPROM.length() ; i+=3) {
        if ((EEPROM[i] & 128) != start){
            //change of flag found
            dataAddress = i - 3;
            mask =  EEPROM[dataAddress] | 127;
            timeCount =  EEPROM[dataAddress++] & 127;
            lapsed = float(timeCount) * 0.1;
            lastLapsed = lapsed;
            battery_a.restoreCharge(double(EEPROM[dataAddress++]) * 0.5);
            battery_b.restoreCharge(double(EEPROM[dataAddress++]) * 0.5);
            break;
        }
      }
  }
  for (uint16_t i = 3 ; i < EEPROM.length()-3 ; i+=3) {
    Serial.print("h");
    Serial.print(i/3);
    Serial.print(":");
    Serial.print(EEPROM[i]);
    Serial.print(",");
    Serial.print(EEPROM[i+1]);
    Serial.print(",");
    Serial.println(EEPROM[i+2]);

  }
  Serial.print("Resored values: addr: ");
  Serial.print(dataAddress);
  Serial.print("time: ");
  Serial.print(timeCount);
  Serial.print("c1: ");
  Serial.print(battery_a.charge);
  Serial.print("c2: ");
  Serial.println(battery_b.charge);

  analogWrite(analogOutPin1, 0);
  delay(2000);
  analogWrite(analogOutPin1, 127);
  delay(1000);
  analogWrite(analogOutPin1, 255);
  delay(1000);
  analogWrite(analogOutPin1, 0);
  
  previousMillis = millis();
  previousZeroMillis = previousMillis;
}


void loop(){
  
    averageAnalogue(0, 10);  
    batteryVolts = scale(sensorTotal[0], milliBatteryVoltRange);

    // When battery > 14.1 volts assume full charge

    if (batteryVolts > 14100){
      battery_a.charge = maxCapacity;
      battery_b.charge = maxCapacity;
      logCharge();
    }
  
    if (batteryVolts < 12.0){
      battery_a.charge = 0.1;
      battery_b.charge = 0.1;
      logCharge();
    }
    
    currentMillis = millis();
    lapsedMillis = currentMillis - previousMillis;
    previousMillis = currentMillis;
    
    lapsedHrs = (double) lapsedMillis /3600000.0;
    battery_a.updateCharge(lapsedHrs);
    battery_b.updateCharge(lapsedHrs);

    if(battery_a.logRequired()){
      logCharge();
    }



    /*
    if (current1 < 0 && currentCharge1 < batteryCapacity){
      // charging
      currentCharge1 -= ((double) current1 * chargeEfficiency * lapsedHrs);
    }

    if (current1 > 0 && currentCharge1 > 100.0){
      //discharging
      currentCharge1 -= ((double) current1 * lapsedHrs);
      if (currentCharge1 < 100.0){
        currentCharge1 = 100.0;
      }
    }

  
    if (current2 < 0){
      current2 = (current2*104)/100;
    }

    if (current2 < 0 && currentCharge2 < batteryCapacity) {
      // charging
      currentCharge2 -= ((double) current2 * chargeEfficiency * lapsedHrs);
    }

    if (current2 > 0 && currentCharge2 > 100.0) {
      // discharging
      currentCharge2 -= ((double) current2 * lapsedHrs);
      if (currentCharge2 < 100.0){
        currentCharge2 = 100.0;
      }
    }


    if (fabs(currentCharge1 - lastLoggedCharge1) > 490.0){
       logCharge();
    }
    */

    if (out1Status == 0){
      digitalWrite(outPin1, HIGH);
      out1Status = 1;
      // map it to analog out:
      outputValue = (battery_a.charge/maxCapacity) * 255;

    }else{
      out1Status = 0;
      digitalWrite(outPin1, LOW);
      // map it to analog out:
      outputValue = (battery_b.charge/maxCapacity) * 255;

    }

    // change the analog out value:
    analogWrite(analogOutPin1, outputValue);
  

    Serial.print("1: ");
    Serial.print(sensorTotal[0]);
   
    Serial.print("Millis: ");
    Serial.print(lapsedMillis);
    Serial.print(", Charge: ");
    Serial.println(outputValue);

    // print the results to the Serial Monitor:
    Serial.print("time: ");
    Serial.print(timeCount);
    Serial.print(",lapsed:");
    Serial.print(lapsed);
    Serial.print(",B1: ");
    Serial.print(battery_a.charge);
    Serial.print (",B2: ");
    Serial.print(battery_b.charge);
    Serial.print(",C1: ");
    Serial.print(battery_a.current);
    Serial.print(",C2: ");
    Serial.print(battery_b.current);
    Serial.print(",V: ");
    Serial.println(batteryVolts/1000.0); 

    /*Serial.print("V_A: ");
    Serial.print(battery_a.getBusVoltage());
    Serial.print(",Shunt_A_mv: ");
    Serial.print(battery_a.getShuntVoltage_mV());
    Serial.print(",V_B: ");
    Serial.print(battery_b.getBusVoltage());
    Serial.print(",Shunt_B_mv: ");
    Serial.println(battery_b.getShuntVoltage_mV());

    Serial.print("Shunt_A_current: ");
    Serial.print(battery_a.getCurrent());
    Serial.print(",Shunt_B_current: ");
    Serial.print(battery_b.getCurrent());
    */

    delay(3000);
}


//functions:

long scale(long reading, long fullScale) {
  return reading * fullScale / 1023L;
}

void averageAnalogue(int indexIn, int total){
  sensorTotal[indexIn] = 0;
  for (int i = 0; i < total; i++) {
    sensorTotal[indexIn] +=  analogRead(analogIn[indexIn]);
    // wait >2 milliseconds before the next loop for the analog-to-digital
    // converter to settle after the last reading:
    delay(4);
    
  }
  sensorTotal[indexIn] = sensorTotal[indexIn]/total;
  delay(100);
}

void logCharge(){
  if  (battery_a.isLogRequired() || battery_b.isLogRequired()){
    if(dataAddress > EEPROM.length() - 4){
      dataAddress = 3;
      mask ^= 128;
      EEPROM[2] = mask;
    }
    Serial.print("log: mask:");
    Serial.print(mask);
    Serial.print(",time:");
    Serial.print(timeCount);
    Serial.print(",lapsed:");
    Serial.print(lapsed);
    Serial.print(",C1:");
    Serial.print(battery_a.charge);
    Serial.print(",C2:");
    Serial.println(battery_b.charge);

    EEPROM[dataAddress++] = (mask & 128) | timeCount;
    EEPROM[dataAddress++] = byte(battery_a.charge*2 + 0.25);
    EEPROM[dataAddress++] = byte(battery_b.charge*2 + 0.25);
    battery_a.chargeLogged();
    battery_b.chargeLogged();
  }
}
