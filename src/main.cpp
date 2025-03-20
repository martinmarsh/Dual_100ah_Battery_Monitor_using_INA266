#include <Arduino.h>
#include <EEPROM.h>

//function declarations:
long scale(long, long);
void averageAnalogue(int, int);
long getCurrent(int);
void logCharge();

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

constexpr  int16_t  epromID = 2345;
constexpr float aref = 1.077;         // ref voltage for 1023 full scale conversion

//calibration voltage measurement
constexpr float voltBatteryRatio = 0.0445509;
constexpr long  milliBatteryVoltRange = aref / voltBatteryRatio  * 1000.0;

//calibration shunt measurements
constexpr float milliVoltOffset1 = 16.15;       //milli volts offsets shunt 1 an 2
constexpr float milliVoltOffset2 = 16.4;  
constexpr float sensitivity1 = .6;   // milli volts per amp shunt 1
constexpr float sensitivity2 = .6;

constexpr float ampsOffset1 = milliVoltOffset1 / sensitivity1;
constexpr float ampsOffset2 = milliVoltOffset2 / sensitivity2;
constexpr float firstStageGain = 5.0;  //Both amps are set to this
constexpr float secondStageGain = 4.0;  //Both amps are set to this

constexpr float ampsFullScale1Hi = ampsOffset1 * aref / milliVoltOffset1 / firstStageGain * 1000.0;
constexpr float ampsFullScale2Hi = ampsOffset2 * aref / milliVoltOffset2 / firstStageGain * 1000.0;

//Total gain allows about -26A to about 62A 
constexpr float ampsFullScale1Lo = ampsOffset1 * aref / milliVoltOffset1 / (firstStageGain * secondStageGain) *1000.0;
constexpr float ampsFullScale2Lo = ampsOffset1 * aref / milliVoltOffset1 / (firstStageGain * secondStageGain) *1000.0;

// Computed values in milliamps to use as long integers with +/- comp for nearest digit to get zero reading
constexpr long milliAmpsOffset1Hi = ampsOffset1 * 1000.0 - 0;
constexpr long milliAmpsOffset2Hi = ampsOffset2 * 1000.0 - 0;
constexpr long milliAmpsOffset1Lo = ampsOffset1 * 1000.0 - 0;
constexpr long milliAmpsOffset2Lo = ampsOffset2 * 1000.0 - 0;

constexpr long milliAmpsFullScale1Hi = ampsFullScale1Hi * 1000.0;
constexpr long milliAmpsFullScale2Hi = ampsFullScale2Hi * 1000.0;
constexpr long milliAmpsFullScale1Lo = ampsFullScale1Lo * 1000.0;
constexpr long milliAmpsFullScale2LO = ampsFullScale2Lo * 1000.0;

constexpr long max1hi = milliAmpsFullScale1Hi - milliAmpsOffset1Hi;
constexpr long max1Lo = milliAmpsFullScale1Lo - milliAmpsOffset1Lo;
constexpr long max2Hi = milliAmpsFullScale2Hi - milliAmpsOffset2Hi;
constexpr long max2Lo = milliAmpsFullScale2LO - milliAmpsOffset2Lo;

constexpr float count1hi = (float)milliAmpsOffset1Hi/(float)milliAmpsFullScale1Hi * 1023.0;
constexpr float count1lo = (float)milliAmpsOffset1Lo/(float)milliAmpsFullScale1Lo * 1023.0;
constexpr float count2hi = (float)milliAmpsOffset2Hi/(float)milliAmpsFullScale2Hi * 1023.0;
constexpr float count2lo = (float)milliAmpsOffset2Lo/(float)milliAmpsFullScale2LO * 1023.0;


unsigned long  currentMillis = 0;
unsigned long  previousMillis = 0;
unsigned long  lapsedMillis = 0;

long batteryVolts = 0;

long current1;
long current2;

long offset1Hi = 0;
long offset1Lo = 0;
long offset2Hi = 0;
long offset2Lo = 0;

long sensorTotal[] = {0, 0, 0, 0, 0};
int outputValue = 0;  // value output to the PWM (analog out)

int indexIn;
int total = 200; //number of reads to average over

int out1Status = 0; 
int reZero = 0;
long totalCurrent1 = 0;
long totalCurrent2 = 0;

byte mask = 0;
unsigned int dataAddress = 3;
byte timeCount = 0;


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
            timeCount =  EEPROM[dataAddress++] & 127;
            currentCharge1 = float(EEPROM[dataAddress++]) * 500.0;
            currentCharge2 = float(EEPROM[dataAddress++]) * 500.0;
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
  averageAnalogue(1, total);
  offset1Lo = getCurrent(1) - 30;
  delay(1000);
  analogWrite(analogOutPin1, 255);
  averageAnalogue(3, total);
  offset2Lo = getCurrent(2) - 30;
  analogWrite(analogOutPin1, 0);
  delay(1000);
  
  previousMillis = millis();
}


void loop(){
  
    if (reZero > 100){
      totalCurrent1 /= reZero;
      totalCurrent2 /= reZero;
      if ((totalCurrent1 < 270 && totalCurrent1 > -240) || (totalCurrent2 < 270 && totalCurrent2 > -240) ){
          if ((current1 < 400 && current1 > -400) || (current2 < 400 && current2 > -400) ){
              offset1Lo = 0;
              offset1Lo = getCurrent(1) - 30;
              offset2Lo = 0;
              offset2Lo = getCurrent(2) - 30;
          }
      }
      totalCurrent1 = 0;
      totalCurrent2 = 0;
      reZero = 0;
      timeCount++;
      timeCount &= 127;
      if (timeCount == 30 || timeCount == 90){
        logCharge();
      }
    }

    averageAnalogue(0, 10);
    averageAnalogue(1, total);
    averageAnalogue(2, 10);
    averageAnalogue(3, total);
    averageAnalogue(4, 20);
 
    current1 = getCurrent(1);
    totalCurrent1 += current1;
    current2 = getCurrent(2);
    totalCurrent2 += current2;
    ++reZero;
  
    batteryVolts = scale(sensorTotal[4], milliBatteryVoltRange);

    // When battery > 14.1 volts and capacity of battery 1 is less than capacity assume full charge
    // has been reached. It is unlikely battery will be over 14.1 volts and 0.25 Ahrs has been used
    // so should not trigger again until recharged

    if (batteryVolts > 14100 && currentCharge1 <= (batteryCapacity - 250)){
      currentCharge1 = batteryCapacity;
      currentCharge2  = batteryCapacity;
      logCharge();
    }

    // When battery < 12.0 volts and capacity of battery 1 is greater than 10000 (10Ahrs) then set charger to 0
    // provided discharge current is not > 2 amps ie high discharge rate.
    // It is unlikely battery will be less than 12.4 volts and has > 10 Ahrs charge so should not trigger again
    // until discharged again.
    
    if (batteryVolts < 12.0 && currentCharge1 > 10000 && current1 < 2000){
      currentCharge1 = 0;
      currentCharge2  = 0;
      logCharge();
    }
    
    currentMillis = millis();
    lapsedMillis = currentMillis - previousMillis;
    previousMillis = currentMillis;

    if (currentCharge1 > 1.0 && (current1 > 0 || currentCharge1 < batteryCapacity)){
      currentCharge1 -= ((double) current1 * (double) lapsedMillis /3600000.0);
    }

    if (currentCharge2 > 1.0 && (current1 > 0 || currentCharge1 < batteryCapacity)) {
      currentCharge2 -= ((double) current2 * (double) lapsedMillis /3600000.0);
    }

    if (fabs(currentCharge1 - lastLoggedCharge1) > 500.0){
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
    Serial.print(" = ");
    Serial.print(offset1Hi);
    Serial.print(" Low1: ");
    Serial.print(sensorTotal[1]);
    Serial.print(" = ");
    Serial.println(offset1Lo);

    Serial.print("2: ");
    Serial.print(sensorTotal[2]);
    Serial.print(" = ");
    Serial.print(offset2Hi);
    Serial.print(" Low2: ");
    Serial.print(sensorTotal[3]);
    Serial.print(" = ");
    Serial.println(offset2Lo);

    Serial.print("Millis: ");
    Serial.print(lapsedMillis);
    Serial.print(", Charge: ");
    Serial.println(outputValue);

    // print the results to the Serial Monitor:
    Serial.print("time: ");
    Serial.print(timeCount);
    Serial.print(",B1: ");
    Serial.print(currentCharge1/1000.0);
    Serial.print (",B2: ");
    Serial.print(currentCharge2/1000.0);
    Serial.print(",C1: ");
    Serial.print(current1);
    Serial.print(",C2: ");
    Serial.print(current2);
    Serial.print(",V: ");
    Serial.println(batteryVolts); 
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
  delay(10);
}

long getCurrent(int channel){
  long current;
  if (channel == 1 ){
      averageAnalogue(1, total);
      if (sensorTotal[1] < 1022){
          current = scale(sensorTotal[1], milliAmpsFullScale1Lo) - milliAmpsOffset1Lo - offset1Lo;
      } else {
        averageAnalogue(0, total);
        current = scale(sensorTotal[0], milliAmpsFullScale1Hi) - milliAmpsOffset1Hi;
      }
  } else {
    averageAnalogue(3, total);
    if (sensorTotal[3] < 1022){
      current = scale(sensorTotal[3], milliAmpsFullScale2LO) - milliAmpsOffset2Lo - offset2Lo;
    } else {
      averageAnalogue(2, total);
      current = scale(sensorTotal[2], milliAmpsFullScale2Hi) - milliAmpsOffset2Hi;
    }
  }
  return current;
}

void logCharge(){
  if(dataAddress > EEPROM.length() - 4){
    dataAddress = 3;
    mask ^= 128;
    EEPROM[2] = mask;
  }
  Serial.print("log: mask:");
  Serial.print(mask);
  Serial.print(",time:");
  Serial.print(timeCount);
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