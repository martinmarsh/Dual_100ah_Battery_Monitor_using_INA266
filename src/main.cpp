#include <Arduino.h>
#include <EEPROM.h>

//function declarations:
long scale(long, long);
void averageAnalogue(int, int);
long getCurrent(int);
void logCharge();
long countToCurrent(int count, float gain, float shunt, float calibration);

//addresses for each input defined above
const int analogIn[] = {A0,A1,A2,A3,A4}; 

//battery capacity in milli-amp hours:
constexpr double batteryCapacity = 95000.0;
double currentCharge1  = batteryCapacity;
double currentCharge2  = batteryCapacity;
double lastLoggedCharge1 = 0;
double lastLoggedCharge2 = 0;


// Output pins via driver
const int outPin1 = 9;  // Analog output pin that the LED is attached
const int outPin2 = 7;
const int outPin3 = 8;
// Analogue output pins
const int analogOutPin1 = 5;  // Analog output pin - direct output 

constexpr  int16_t  epromID = 2346;
constexpr float aref = 1.077;         // ref voltage for 1023 full scale conversion
constexpr float shuntBat1 = 0.031;   // mv per 100A equivalent as measured including other factors
constexpr float shuntBat2 = 0.090;
constexpr float gain2nd = 15.0;       // gain at 2nd stage total from input stages
constexpr float gain1st = 5.0;        // gain of 1st stage
constexpr float calBattery1 = 1.0;   // apply calibration to current conversions
constexpr float calBattery2 = 1.0;
constexpr float chargeEfficiency = .90;  // Lipo4 charge efficiency - about of charge actually stored

constexpr int idealCount2nd = 0.0166 * gain2nd / 1.077 * 1023.0 + 0.5;  //237

constexpr float idealCurrentOffset2nd = (float) idealCount2nd * aref / 1023 / gain2nd/ shuntBat1 * 100.0;  // 21.34A

constexpr int idealCount1st = 0.0166 * gain1st / 1.077 * 1023.0 + 0.5;  // 76
constexpr float idealCurrentOffset1st = (float) idealCount1st * aref / 1023 / gain1st/ shuntBat1 * 100.0;  //21.34A

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

long current1;
long current2;

long currentOffsetBat1 = 0;
long currentOffsetBat2 = 0;

long sensorTotal[] = {0, 0, 0, 0, 0};
int outputValue = 0;  // value output to the PWM (analog out)

int indexIn;
int total = 200; //default number of reads to average over current readings

int out1Status = 0; 
int reZero = 0;
long totalCurrent1 = 0;
long totalCurrent2 = 0;
int offLoadPeriod = 0;

byte mask = 0;
unsigned int dataAddress = 3;
byte timeCount = 0;
float lastLapsed = 0;
double lapsedHrs = 0;


void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  for (indexIn = 0; indexIn < 5; indexIn++) {
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
      currentCharge1 = batteryCapacity;
      currentCharge2  = batteryCapacity;
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
            currentCharge1 = float(EEPROM[dataAddress++]) * 500.0;
            currentCharge2 = float(EEPROM[dataAddress++]) * 500.0;
            if (currentCharge1 > batteryCapacity){
              currentCharge1 = batteryCapacity;
            }
            if (currentCharge2 > batteryCapacity){
              currentCharge2 = batteryCapacity;
            }
            if (currentCharge1 < 1){
              currentCharge1 = 0.1;
            }
            if (currentCharge2 < 1){
              currentCharge2 = 0.1;
            }
            lastLoggedCharge1 = currentCharge1;
            lastLoggedCharge2 = currentCharge2;
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
  Serial.print(currentCharge1);
  Serial.print("c2: ");
  Serial.println(currentCharge2);

  delay(2000);
  analogWrite(analogOutPin1, 0);
  delay(2000);
  analogWrite(analogOutPin1, 127);
  currentOffsetBat1 = 0;
  currentOffsetBat1 =  getCurrent(1) - 30;
  delay(1000);
  analogWrite(analogOutPin1, 255);
  currentOffsetBat2 = 0;
  currentOffsetBat2 = getCurrent(2) - 30;
  analogWrite(analogOutPin1, 0);
  delay(1000);
  
  previousMillis = millis();
  previousZeroMillis = previousMillis;
}


void loop(){
  
    if (reZero > 20){
      //approx 1.5 x 20 = 30s
      currentMillis = millis();
      lapsedZeroMillis = currentMillis - previousZeroMillis;
      previousZeroMillis = currentMillis;
      lapsed += float(lapsedZeroMillis)/3600000.0;

      totalCurrent1 /= reZero;
      totalCurrent2 /= reZero;
      if ( ((totalCurrent1 < 370 && totalCurrent1 > -340) || (totalCurrent2 < 370 && totalCurrent2 > -340)) && (offLoadPeriod <= 0 ) ){
            currentOffsetBat1 += totalCurrent1 - 30;
            currentOffsetBat2 += totalCurrent2 - 30;
            Serial.println("Offset reset in idle");
      }
      reZero = 0;
      if ((lapsed - lastLapsed) > 0.1){
        timeCount++;
        timeCount &= 127;
        lastLapsed = lapsed;
      }
      Serial.print("TP: ");
      Serial.print(timeCount);
      Serial.print(",lapsed:");
      Serial.print(lapsed);
      Serial.print(",Inuse:");
      Serial.print(offLoadPeriod);
      Serial.print(",TC1:");
      Serial.print(totalCurrent1);
      Serial.print(",TC2:");
      Serial.print(totalCurrent2);
      Serial.print(",offset1:");
      Serial.print(currentOffsetBat1/1000.0);
      Serial.print(",offset2:");
      Serial.println(currentOffsetBat2/1000.0);
      totalCurrent1 = 0;
      totalCurrent2 = 0;
      if (offLoadPeriod > 0){
        --offLoadPeriod;
      }
  
      if (timeCount == 30 || timeCount == 90){
        logCharge();
      }
    }

    averageAnalogue(4, 20);
    averageAnalogue(0, 10);
    averageAnalogue(2, 10);
 
    current1 = getCurrent(1);
    totalCurrent1 += current1;
    current2 = getCurrent(2);
    totalCurrent2 += current2;
    ++reZero;
  
    batteryVolts = scale(sensorTotal[4], milliBatteryVoltRange);

    // When battery > 14.1 volts and capacity of battery 1 is less than capacity assume full charge
    // has been reached. 

    if (batteryVolts > 14100){
      currentCharge1 = batteryCapacity;
      currentCharge2  = batteryCapacity;
      logCharge();
    }

    // When battery < 12.0 volts and capacity of battery 1 is greater than 10000 (10Ahrs) then set charger to 0
    // provided discharge current is not > 2 amps ie high discharge rate.
    
    if (batteryVolts < 12.0){
      currentCharge1 = 100.0;
      currentCharge2  = 100.0;
      logCharge();
    }
    
    currentMillis = millis();
    lapsedMillis = currentMillis - previousMillis;
    previousMillis = currentMillis;

    //if (current1 < 0){
       //current1 = current1*90/100;
    // }

    if ((current1 > 400 && current2 > 400) || (current1 < -400 && current2 < -400) ){
        offLoadPeriod = 1;    // do not reset zero after this period ends as average for period will be affected by use
    }
    if ((current1 > 3000 && current2 > 3000) || (current1 < -3000 && current2 < -3000) ){
      offLoadPeriod = 20;      // skip next n periods of approx 30s ie 20 = 10mins to let batteries balance
    }

    lapsedHrs = (double) lapsedMillis /3600000.0;
    
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

  
    //if (current2 < 0){
      //current2 = current2*94/100;
    //}

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


    if (out1Status == 0){
      digitalWrite(outPin1, HIGH);
      out1Status = 1;
      // map it to analog out:
      outputValue = (currentCharge1/batteryCapacity) * 255;

    }else{
      out1Status = 0;
      digitalWrite(outPin1, LOW);
      // map it to analog out:
      outputValue = (currentCharge2/batteryCapacity) * 255;

    }

    // change the analog out value:
    analogWrite(analogOutPin1, outputValue);
  

    Serial.print("1: ");
    Serial.print(sensorTotal[0]);
    Serial.print(" -- ");
    Serial.print(sensorTotal[1]);
    Serial.print(" offset ");
    Serial.println(currentOffsetBat1/1000.0);

    Serial.print("2: ");
    Serial.print(sensorTotal[2]);
    Serial.print(" -- ");
    Serial.print(sensorTotal[3]);
    Serial.print(" offSet ");
    Serial.println(currentOffsetBat2/1000.0);

    Serial.print("Millis: ");
    Serial.print(lapsedMillis);
    Serial.print(", Charge: ");
    Serial.println(outputValue);

    // print the results to the Serial Monitor:
    Serial.print("time: ");
    Serial.print(timeCount);
    Serial.print(",lapsed:");
    Serial.print(lapsed);
    Serial.print(",Inuse:");
    Serial.print(offLoadPeriod);
    Serial.print(",B1: ");
    Serial.print(currentCharge1/1000.0);
    Serial.print (",B2: ");
    Serial.print(currentCharge2/1000.0);
    Serial.print(",C1: ");
    Serial.print(current1/1000.0);
    Serial.print(",C2: ");
    Serial.print(current2/1000.0);
    Serial.print(",V: ");
    Serial.println(batteryVolts/1000.0); 
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
    delay(2);
    
  }
  sensorTotal[indexIn] = sensorTotal[indexIn]/total;
  delay(100);
}

long getCurrent(int battery){
  long current;
  if (battery == 1 ){
      averageAnalogue(1, total);
      if (sensorTotal[1] < 1022){
          current = countToCurrent(sensorTotal[1], gain2nd, shuntBat1, calBattery1) - currentOffsetBat1;
      } else {
        averageAnalogue(0, total);
        current = countToCurrent(sensorTotal[0], gain1st, shuntBat1, calBattery1) - currentOffsetBat1;
      }
  } else {
    averageAnalogue(3, total);
    if (sensorTotal[3] < 1022){
      current = countToCurrent(sensorTotal[3], gain2nd, shuntBat2, calBattery2) - currentOffsetBat2;
    } else {
      averageAnalogue(2, total);
      current = countToCurrent(sensorTotal[2], gain1st, shuntBat2, calBattery2) - currentOffsetBat2;
    }
  }
  return current;
}

void logCharge(){
  if  (lastLoggedCharge1 != currentCharge1 || lastLoggedCharge2 != currentCharge2){
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
    Serial.print(currentCharge1);
    Serial.print(",C2:");
    Serial.println(currentCharge2);

    EEPROM[dataAddress++] = (mask & 128) | timeCount;
    EEPROM[dataAddress++] = byte(currentCharge1/500.0 + 0.25);
    EEPROM[dataAddress++] = byte(currentCharge2/500.0 + 0.25);
    lastLoggedCharge1 = currentCharge1;
    lastLoggedCharge2 = currentCharge2;
  }
}


long countToCurrent(int count, float gain, float shunt, float calibration){
  return  (float) count * aref / 1023.0 / gain/ shunt * 100000.0 * calibration;

}
